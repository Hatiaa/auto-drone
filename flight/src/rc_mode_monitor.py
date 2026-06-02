import argparse
import time

from pymavlink import mavutil


DEFAULT_DEVICE = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600


def switch_label(value):
    if value is None or value == 0:
        return "--"
    if value < 1300:
        return "LOW"
    if value > 1700:
        return "HIGH"
    return "MID"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--start-threshold", type=int, default=1700)
    parser.add_argument("--print-period", type=float, default=0.3)
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.device, baud=args.baud)
    master.wait_heartbeat(timeout=15)
    print("[connect] connected system=%s component=%s" % (master.target_system, master.target_component))

    mode = "--"
    armed = False
    ch5 = 0
    ch6 = 0
    ch7 = 0
    last_print = 0

    while True:
        msg = master.recv_match(
            type=["HEARTBEAT", "RC_CHANNELS", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            print("[status] severity=%s text=%s" % (getattr(msg, "severity", "--"), getattr(msg, "text", "")))
            continue

        if msg.get_type() == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            armed = bool(int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            continue

        if msg.get_type() == "RC_CHANNELS":
            ch5 = getattr(msg, "chan5_raw", 0)
            ch6 = getattr(msg, "chan6_raw", 0)
            ch7 = getattr(msg, "chan7_raw", 0)

        now = time.time()
        if now - last_print < args.print_period:
            continue
        last_print = now

        control_allowed = (mode == "GUIDED" and ch6 > args.start_threshold)
        arm_switch_high = (ch7 > args.start_threshold)
        flight_ready = (control_allowed and arm_switch_high)

        print(
            "mode=%-10s armed=%-5s | "
            "CH5=%4s %-4s CH6=%4s %-4s CH7=%4s %-4s | "
            "control_allowed=%s arm_switch=%s flight_ready=%s"
            % (
                mode,
                str(armed),
                ch5,
                switch_label(ch5),
                ch6,
                switch_label(ch6),
                ch7,
                switch_label(ch7),
                "YES" if control_allowed else "NO",
                "YES" if arm_switch_high else "NO",
                "YES" if flight_ready else "NO",
            )
        )


if __name__ == "__main__":
    main()
