import argparse
import time

from pymavlink import mavutil


DEFAULT_DEVICE = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600
THROTTLE_CHANNEL_INDEX = 2  # RC channel 3, zero-based.


def connect(device, baud, timeout_s):
    print("[connect] device=%s baud=%s" % (device, baud))
    master = mavutil.mavlink_connection(device, baud=baud)
    print("[connect] waiting for heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise TimeoutError("No heartbeat received from Pixhawk")
    print(
        "[connect] heartbeat received system=%s component=%s type=%s autopilot=%s"
        % (
            master.target_system,
            master.target_component,
            getattr(heartbeat, "type", "--"),
            getattr(heartbeat, "autopilot", "--"),
        )
    )
    return master


def set_mode(master, mode, timeout_s):
    print("[mode] setting %s" % mode)
    mode_mapping = master.mode_mapping()
    mode_id = mode_mapping.get(mode)
    if mode_id is None:
        raise RuntimeError("Mode %r is not available. Available modes: %s" % (mode, sorted(mode_mapping.keys())))

    master.set_mode(mode_id)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        current = mavutil.mode_string_v10(msg)
        armed = bool(int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print("[mode] current=%s armed=%s" % (current, armed))
        if current.upper() == mode.upper():
            return
    raise TimeoutError("Timed out waiting for mode %s" % mode)


def arm(master, timeout_s):
    print("[arm] arming motors")
    master.arducopter_arm()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=["HEARTBEAT", "STATUSTEXT"],
            blocking=True,
            timeout=0.5,
        )
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("[status] severity=%s text=%s" % (getattr(msg, "severity", "--"), getattr(msg, "text", "")))
            continue
        armed = bool(int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print("[arm] waiting armed=%s" % armed)
        if armed:
            print("[arm] armed")
            return
    raise TimeoutError("Timed out waiting for armed heartbeat")


def disarm(master):
    print("[disarm] disarming motors")
    master.arducopter_disarm()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        armed = bool(int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if not armed:
            print("[disarm] disarmed")
            return
    print("[disarm] disarm wait timed out; check vehicle state manually")


def throttle_percent_to_pwm(percent, min_pwm, max_pwm):
    return int(round(min_pwm + (max_pwm - min_pwm) * (percent / 100.0)))


def send_rc_override(master, throttle_pwm):
    # Channels 1/2/4 neutral, channel 3 throttle. Other channels ignored.
    channels8 = [1500, 1500, throttle_pwm, 1500, 65535, 65535, 65535, 65535]
    channels18 = channels8 + [65535] * 10

    try:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *channels18
        )
    except TypeError:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *channels8
        )


def release_rc_override(master):
    channels8 = [0] * 8
    channels18 = [0] * 18
    try:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *channels18
        )
    except TypeError:
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *channels8
        )


def print_status_messages(master, duration_s):
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=["STATUSTEXT", "HEARTBEAT", "SYS_STATUS"],
            blocking=True,
            timeout=0.2,
        )
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("[status] severity=%s text=%s" % (getattr(msg, "severity", "--"), getattr(msg, "text", "")))
        elif msg.get_type() == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            armed = bool(int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print("[heartbeat] mode=%s armed=%s" % (mode, armed))
        elif msg.get_type() == "SYS_STATUS":
            voltage_mv = int(getattr(msg, "voltage_battery", -1))
            if voltage_mv > 0:
                print("[battery] %.2fV" % (voltage_mv / 1000.0))


def run(args):
    if not args.props_removed:
        raise SystemExit(
            "Refusing to run simultaneous motor test until --props-removed is provided. "
            "Remove all propellers first."
        )

    if args.throttle != 20.0:
        raise SystemExit("This script is intentionally fixed for 20%% throttle. Use --throttle 20.")

    if args.duration != 10.0:
        raise SystemExit("This script is intentionally fixed for 10 seconds. Use --duration 10.")

    throttle_pwm = throttle_percent_to_pwm(args.throttle, args.min_pwm, args.max_pwm)
    idle_pwm = args.min_pwm

    print("[safety] PROPELLERS MUST BE REMOVED")
    print("[test] simultaneous motor test: throttle=%.1f%% pwm=%d duration=%.1fs" % (
        args.throttle,
        throttle_pwm,
        args.duration,
    ))

    master = connect(args.device, args.baud, args.heartbeat_timeout)

    try:
        set_mode(master, args.mode, timeout_s=args.mode_timeout)

        # Send low throttle before arming so the autopilot sees a sane throttle input.
        for _ in range(5):
            send_rc_override(master, idle_pwm)
            time.sleep(0.1)

        arm(master, timeout_s=args.arm_timeout)
        print_status_messages(master, duration_s=1.0)

        print("[motor-test] starting all motors now")
        deadline = time.monotonic() + args.duration
        next_print = 0.0
        while time.monotonic() < deadline:
            send_rc_override(master, throttle_pwm)
            now = time.monotonic()
            if now >= next_print:
                print("[motor-test] throttle override pwm=%d" % throttle_pwm)
                next_print = now + 1.0
            time.sleep(0.05)

        print("[motor-test] stopping motors")
        for _ in range(20):
            send_rc_override(master, idle_pwm)
            time.sleep(0.05)

    finally:
        release_rc_override(master)
        disarm(master)

    print("[result] simultaneous motor test complete")


def parse_args():
    parser = argparse.ArgumentParser(
        description="No-prop simultaneous motor test: arm in STABILIZE and hold 20 percent throttle for 10 seconds."
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--mode", default="STABILIZE")
    parser.add_argument("--throttle", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--min-pwm", type=int, default=1000)
    parser.add_argument("--max-pwm", type=int, default=2000)
    parser.add_argument("--heartbeat-timeout", type=float, default=15.0)
    parser.add_argument("--mode-timeout", type=float, default=10.0)
    parser.add_argument("--arm-timeout", type=float, default=10.0)
    parser.add_argument("--props-removed", action="store_true")
    return parser.parse_args()


def main():
    try:
        run(parse_args())
    except KeyboardInterrupt:
        print("\n[exit] interrupted by user")


if __name__ == "__main__":
    main()
