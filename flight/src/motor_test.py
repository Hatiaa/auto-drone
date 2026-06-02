import argparse
import time

from pymavlink import mavutil


DEFAULT_DEVICE = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600

MAV_CMD_DO_MOTOR_TEST = getattr(mavutil.mavlink, "MAV_CMD_DO_MOTOR_TEST", 209)
MAV_RESULT_ACCEPTED = getattr(mavutil.mavlink, "MAV_RESULT_ACCEPTED", 0)
MOTOR_TEST_THROTTLE_PERCENT = 0
MOTOR_TEST_ORDER_DEFAULT = 0
MOTOR_TEST_COMPONENT = 1

MAV_RESULT_NAMES = {
    getattr(mavutil.mavlink, "MAV_RESULT_ACCEPTED", 0): "ACCEPTED",
    getattr(mavutil.mavlink, "MAV_RESULT_TEMPORARILY_REJECTED", 1): "TEMPORARILY_REJECTED",
    getattr(mavutil.mavlink, "MAV_RESULT_DENIED", 2): "DENIED",
    getattr(mavutil.mavlink, "MAV_RESULT_UNSUPPORTED", 3): "UNSUPPORTED",
    getattr(mavutil.mavlink, "MAV_RESULT_FAILED", 4): "FAILED",
    getattr(mavutil.mavlink, "MAV_RESULT_IN_PROGRESS", 5): "IN_PROGRESS",
    getattr(mavutil.mavlink, "MAV_RESULT_CANCELLED", 6): "CANCELLED",
}


def _connect(device: str, baud: int, heartbeat_timeout_s: float):
    print(f"[connect] device={device} baud={baud}")
    master = mavutil.mavlink_connection(device, baud=baud)

    print("[connect] waiting for heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=heartbeat_timeout_s)
    if heartbeat is None:
        raise TimeoutError("No heartbeat received from Pixhawk")

    print(
        "[connect] heartbeat received "
        f"system={master.target_system} component={master.target_component} "
        f"type={getattr(heartbeat, 'type', '--')} autopilot={getattr(heartbeat, 'autopilot', '--')}"
    )
    return master


def _request_data_stream(master, stream_id: int, rate_hz: int) -> None:
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        stream_id,
        rate_hz,
        1,
    )


def _print_precheck_messages(master, wait_s: float) -> None:
    _request_data_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)

    print(f"[precheck] listening for status for {wait_s:.1f}s...")
    deadline = time.monotonic() + wait_s
    saw_status = False
    while time.monotonic() < deadline:
        remaining_s = max(0.0, deadline - time.monotonic())
        msg = master.recv_match(
            type=["SYS_STATUS", "STATUSTEXT", "HEARTBEAT"],
            blocking=True,
            timeout=remaining_s,
        )
        if msg is None:
            break

        msg_type = msg.get_type()
        if msg_type == "SYS_STATUS":
            saw_status = True
            voltage = getattr(msg, "voltage_battery", -1)
            current = getattr(msg, "current_battery", -1)
            print(
                "[precheck] SYS_STATUS "
                f"voltage={voltage / 1000.0:.2f}V "
                f"current={current / 100.0:.2f}A "
                f"errors_count1={getattr(msg, 'errors_count1', '--')}"
            )
        elif msg_type == "STATUSTEXT":
            severity = getattr(msg, "severity", "--")
            text = getattr(msg, "text", "")
            print(f"[status] severity={severity} text={text}")
        elif msg_type == "HEARTBEAT":
            base_mode = int(getattr(msg, "base_mode", 0))
            armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(
                "[precheck] HEARTBEAT "
                f"armed={armed} mode={mavutil.mode_string_v10(msg)} "
                f"system_status={getattr(msg, 'system_status', '--')}"
            )

    if not saw_status:
        print("[precheck] no SYS_STATUS seen; continuing anyway")


def _wait_motor_test_ack(master, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining_s = max(0.0, deadline - time.monotonic())
        msg = master.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT"],
            blocking=True,
            timeout=remaining_s,
        )
        if msg is None:
            break

        if msg.get_type() == "STATUSTEXT":
            severity = getattr(msg, "severity", "--")
            text = getattr(msg, "text", "")
            print(f"[status] severity={severity} text={text}")
            continue

        if int(getattr(msg, "command", -1)) != MAV_CMD_DO_MOTOR_TEST:
            continue

        result = int(getattr(msg, "result", -1))
        result_name = MAV_RESULT_NAMES.get(result, f"UNKNOWN({result})")
        print(f"[ack] MAV_CMD_DO_MOTOR_TEST result={result_name}")
        return result == MAV_RESULT_ACCEPTED

    print("[ack] no COMMAND_ACK received for MAV_CMD_DO_MOTOR_TEST")
    return False


def run(args: argparse.Namespace) -> None:
    if not args.props_removed:
        raise SystemExit(
            "Refusing to run motor test until --props-removed is provided. "
            "Remove all propellers first."
        )

    if not 1 <= args.motor <= 8:
        raise SystemExit("--motor must be between 1 and 8")

    if not 0 <= args.throttle <= 20:
        raise SystemExit("--throttle is limited to 0..20 percent for this safety test")

    if not 0.1 <= args.duration <= 5.0:
        raise SystemExit("--duration is limited to 0.1..5.0 seconds")

    master = _connect(args.device, args.baud, args.heartbeat_timeout)
    _print_precheck_messages(master, args.precheck_wait)

    print(
        "[motor-test] "
        f"motor={args.motor} throttle={args.throttle:.1f}% "
        f"duration={args.duration:.1f}s count=1"
    )

    master.mav.command_long_send(
        master.target_system,
        args.component,
        MAV_CMD_DO_MOTOR_TEST,
        0,
        float(args.motor),
        float(MOTOR_TEST_THROTTLE_PERCENT),
        float(args.throttle),
        float(args.duration),
        1.0,
        float(MOTOR_TEST_ORDER_DEFAULT),
        0.0,
    )

    accepted = _wait_motor_test_ack(master, args.ack_timeout)
    if accepted:
        print("[result] accepted. Motor should stop automatically after duration.")
    else:
        raise SystemExit("[result] command was not accepted or no ack was received")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a short ArduPilot motor test from Jetson Nano via MAVLink."
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="Serial device on Nano")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--motor", type=int, required=True, help="Motor number to test")
    parser.add_argument(
        "--throttle",
        type=float,
        default=5.0,
        help="Throttle percent for the motor test. Limited to 0..20.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="Motor test duration in seconds. Limited to 0.1..5.0.",
    )
    parser.add_argument(
        "--heartbeat-timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for Pixhawk heartbeat.",
    )
    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for command acknowledgement.",
    )
    parser.add_argument(
        "--precheck-wait",
        type=float,
        default=2.0,
        help="Seconds to print heartbeat/status text before sending motor test.",
    )
    parser.add_argument(
        "--component",
        type=int,
        default=MOTOR_TEST_COMPONENT,
        help="MAVLink target component. ArduPilot motor test usually uses component 1.",
    )
    parser.add_argument(
        "--props-removed",
        action="store_true",
        help="Required safety flag. Confirms all propellers have been removed.",
    )
    return parser.parse_args()


def main() -> None:
    try:
        run(parse_args())
    except KeyboardInterrupt:
        print("\n[exit] interrupted by user")


if __name__ == "__main__":
    main()
