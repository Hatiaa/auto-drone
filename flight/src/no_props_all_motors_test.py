import argparse
import time

from pymavlink import mavutil


DEFAULT_DEVICE = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600
AUTOPILOT_COMPONENT = 1
MAV_CMD_DO_MOTOR_TEST = getattr(mavutil.mavlink, "MAV_CMD_DO_MOTOR_TEST", 209)
MOTOR_TEST_THROTTLE_PERCENT = 0
MOTOR_TEST_ORDER_DEFAULT = 0

MAV_RESULT_NAMES = {
    getattr(mavutil.mavlink, "MAV_RESULT_ACCEPTED", 0): "ACCEPTED",
    getattr(mavutil.mavlink, "MAV_RESULT_TEMPORARILY_REJECTED", 1): "TEMPORARILY_REJECTED",
    getattr(mavutil.mavlink, "MAV_RESULT_DENIED", 2): "DENIED",
    getattr(mavutil.mavlink, "MAV_RESULT_UNSUPPORTED", 3): "UNSUPPORTED",
    getattr(mavutil.mavlink, "MAV_RESULT_FAILED", 4): "FAILED",
    getattr(mavutil.mavlink, "MAV_RESULT_IN_PROGRESS", 5): "IN_PROGRESS",
    getattr(mavutil.mavlink, "MAV_RESULT_CANCELLED", 6): "CANCELLED",
}


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


def wait_motor_ack(master, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=max(0.0, deadline - time.monotonic()),
        )
        if msg is None:
            break

        if msg.get_type() == "STATUSTEXT":
            print("[status] severity=%s text=%s" % (getattr(msg, "severity", "--"), getattr(msg, "text", "")))
            continue

        if int(getattr(msg, "command", -1)) != MAV_CMD_DO_MOTOR_TEST:
            continue

        result = int(getattr(msg, "result", -1))
        print("[ack] MAV_CMD_DO_MOTOR_TEST result=%s" % MAV_RESULT_NAMES.get(result, "UNKNOWN(%s)" % result))
        return result == mavutil.mavlink.MAV_RESULT_ACCEPTED

    print("[ack] no COMMAND_ACK received for MAV_CMD_DO_MOTOR_TEST")
    return False


def run_motor(master, motor, throttle, duration):
    print("[motor-test] motor=%d throttle=%.1f%% duration=%.1fs" % (motor, throttle, duration))
    master.mav.command_long_send(
        master.target_system,
        AUTOPILOT_COMPONENT,
        MAV_CMD_DO_MOTOR_TEST,
        0,
        float(motor),
        float(MOTOR_TEST_THROTTLE_PERCENT),
        float(throttle),
        float(duration),
        1.0,
        float(MOTOR_TEST_ORDER_DEFAULT),
        0.0,
    )
    if not wait_motor_ack(master, timeout_s=5.0):
        raise RuntimeError("Motor %d test was not accepted" % motor)


def run(args):
    if not args.props_removed:
        raise SystemExit(
            "Refusing to run all-motor test until --props-removed is provided. "
            "Remove all propellers first."
        )

    if args.throttle != 20.0:
        raise SystemExit("This script is intentionally fixed for 20%% throttle. Use --throttle 20.")

    if args.duration != 10.0:
        raise SystemExit("This script is intentionally fixed for 10 seconds. Use --duration 10.")

    master = connect(args.device, args.baud, args.heartbeat_timeout)

    print("[safety] props_removed=True")
    print("[test] sequentially testing motors 1, 2, 3, 4 at 20%% for 10s each")
    print("[test] press Ctrl+C to abort between motors")

    for motor in [1, 2, 3, 4]:
        run_motor(master, motor, args.throttle, args.duration)
        print("[motor-test] motor=%d command accepted; waiting %.1fs before next motor" % (motor, args.pause))
        time.sleep(args.duration + args.pause)

    print("[result] all four motor test commands accepted")


def parse_args():
    parser = argparse.ArgumentParser(
        description="No-prop sequential test: motors 1-4 at 20 percent for 10 seconds each."
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--throttle", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--pause", type=float, default=2.0)
    parser.add_argument("--heartbeat-timeout", type=float, default=15.0)
    parser.add_argument("--props-removed", action="store_true")
    return parser.parse_args()


def main():
    try:
        run(parse_args())
    except KeyboardInterrupt:
        print("\n[exit] interrupted by user")


if __name__ == "__main__":
    main()
