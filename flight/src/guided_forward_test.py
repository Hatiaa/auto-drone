import argparse
import math
import time

from pymavlink import mavutil


DEFAULT_DEVICE = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600
GUIDED_MODE = "GUIDED"
ALTITUDE_REACHED_TOLERANCE_M = 0.10
ALTITUDE_STABLE_SECONDS = 0.8
POSITION_REACHED_TOLERANCE_M = 0.15
POSITION_STABLE_SECONDS = 0.8
FORWARD_COMMAND_RATE_HZ = 10
CROSSTRACK_GAIN = 0.8
ALTITUDE_HOLD_GAIN = 0.6
DEFAULT_MAX_LATERAL_SPEED_MPS = 0.12
DEFAULT_MAX_VERTICAL_SPEED_MPS = 0.15
DEFAULT_MAX_CROSSTRACK_ERROR_M = 0.35
DEFAULT_MAX_ALTITUDE_ERROR_M = 0.25
DEFAULT_MAX_ROLL_PITCH_DEG = 18.0
DEFAULT_MAX_YAW_ERROR_DEG = 25.0
DEFAULT_MIN_ALTITUDE_TIMEOUT_S = 12.0
DEFAULT_MIN_FORWARD_TIMEOUT_S = 8.0
DEFAULT_PRE_HOLD_S = 2.0
DEFAULT_POST_HOLD_S = 2.0
DEFAULT_LAND_MONITOR_S = 12.0
DEFAULT_START_THRESHOLD = 1700
DEFAULT_HEARTBEAT_TIMEOUT_S = 15.0
MAVLINK_RANGEFINDER_ID = 173
RANGEFINDER_RATE_HZ = 10


class Snapshot:
    def __init__(self):
        self.mode = "--"
        self.armed = False
        self.ch6 = 0
        self.ch7 = 0
        self.range_m = None
        self.range_source = "--"
        self.rel_alt_m = None
        self.local_x_m = None
        self.local_y_m = None
        self.local_z_m = None
        self.roll_deg = None
        self.pitch_deg = None
        self.yaw_rad = None
        self.status_text = ""


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


def request_stream(master, stream_id, rate_hz):
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        stream_id,
        rate_hz,
        1,
    )


def request_message_interval(master, message_id, rate_hz):
    interval_us = int(1000000 / rate_hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavlink_const("MAV_CMD_SET_MESSAGE_INTERVAL", 511),
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def request_basic_streams(master):
    request_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5)
    request_stream(master, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10)
    request_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
    extra2_stream = getattr(mavutil.mavlink, "MAV_DATA_STREAM_EXTRA2", None)
    if extra2_stream is not None:
        request_stream(master, extra2_stream, 10)
    extra3_stream = getattr(mavutil.mavlink, "MAV_DATA_STREAM_EXTRA3", None)
    if extra3_stream is not None:
        request_stream(master, extra3_stream, RANGEFINDER_RATE_HZ)
    request_message_interval(master, MAVLINK_RANGEFINDER_ID, RANGEFINDER_RATE_HZ)
    request_stream(master, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10)


def update_snapshot(snapshot, msg):
    msg_type = msg.get_type()
    if msg_type == "HEARTBEAT":
        snapshot.mode = mavutil.mode_string_v10(msg)
        snapshot.armed = bool(
            int(getattr(msg, "base_mode", 0)) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
    elif msg_type == "RC_CHANNELS":
        snapshot.ch6 = int(getattr(msg, "chan6_raw", 0))
        snapshot.ch7 = int(getattr(msg, "chan7_raw", 0))
    elif msg_type == "RANGEFINDER":
        distance_m = float(getattr(msg, "distance", 0.0))
        if distance_m > 0:
            snapshot.range_m = distance_m
            snapshot.range_source = "RANGEFINDER"
    elif msg_type == "GLOBAL_POSITION_INT":
        snapshot.rel_alt_m = float(getattr(msg, "relative_alt", 0)) / 1000.0
    elif msg_type == "LOCAL_POSITION_NED":
        snapshot.local_x_m = float(getattr(msg, "x", 0.0))
        snapshot.local_y_m = float(getattr(msg, "y", 0.0))
        snapshot.local_z_m = float(getattr(msg, "z", 0.0))
    elif msg_type == "ATTITUDE":
        snapshot.roll_deg = math.degrees(float(getattr(msg, "roll", 0.0)))
        snapshot.pitch_deg = math.degrees(float(getattr(msg, "pitch", 0.0)))
        snapshot.yaw_rad = float(getattr(msg, "yaw", 0.0))
    elif msg_type == "STATUSTEXT":
        snapshot.status_text = getattr(msg, "text", "")
        print("[status] severity=%s text=%s" % (getattr(msg, "severity", "--"), snapshot.status_text))


def read_messages(master, snapshot, duration_s, print_period_s):
    deadline = time.monotonic() + duration_s
    next_print = 0.0
    while time.monotonic() < deadline:
        msg = master.recv_match(
            type=[
                "HEARTBEAT",
                "RC_CHANNELS",
                "RANGEFINDER",
                "GLOBAL_POSITION_INT",
                "LOCAL_POSITION_NED",
                "ATTITUDE",
                "STATUSTEXT",
            ],
            blocking=True,
            timeout=0.2,
        )
        if msg is not None:
            update_snapshot(snapshot, msg)

        now = time.monotonic()
        if now >= next_print:
            print_snapshot(snapshot)
            next_print = now + print_period_s


def print_snapshot(snapshot):
    altitude = get_altitude(snapshot)
    alt_text = "--" if altitude is None else "%.2fm[%s]" % (altitude, snapshot.range_source)
    rel_alt_text = "--" if snapshot.rel_alt_m is None else "%.2fm" % snapshot.rel_alt_m
    local_text = (
        "--"
        if snapshot.local_x_m is None or snapshot.local_y_m is None
        else "x=%.2fm y=%.2fm" % (snapshot.local_x_m, snapshot.local_y_m)
    )
    roll_text = "--" if snapshot.roll_deg is None else "%.1fdeg" % snapshot.roll_deg
    pitch_text = "--" if snapshot.pitch_deg is None else "%.1fdeg" % snapshot.pitch_deg
    yaw_text = "--" if snapshot.yaw_rad is None else "%.1fdeg" % math.degrees(snapshot.yaw_rad)
    print(
        "[telemetry] mode=%s armed=%s CH6=%s CH7=%s range=%s rel_alt=%s local=%s roll=%s pitch=%s yaw=%s"
        % (
            snapshot.mode,
            snapshot.armed,
            snapshot.ch6,
            snapshot.ch7,
            alt_text,
            rel_alt_text,
            local_text,
            roll_text,
            pitch_text,
            yaw_text,
        )
    )


def control_allowed(snapshot, threshold):
    return snapshot.mode == GUIDED_MODE and snapshot.ch6 > threshold and snapshot.ch7 > threshold


def wait_for_control_allowed(master, snapshot, threshold):
    print("[gate] waiting for GUIDED + CH6 high + CH7 high")
    next_print = 0.0
    while True:
        msg = master.recv_match(
            type=[
                "HEARTBEAT",
                "RC_CHANNELS",
                "RANGEFINDER",
                "GLOBAL_POSITION_INT",
                "LOCAL_POSITION_NED",
                "ATTITUDE",
                "STATUSTEXT",
            ],
            blocking=True,
            timeout=0.2,
        )
        if msg is not None:
            update_snapshot(snapshot, msg)

        if control_allowed(snapshot, threshold):
            print("[gate] control allowed")
            print_snapshot(snapshot)
            return

        now = time.monotonic()
        if now >= next_print:
            print_snapshot(snapshot)
            next_print = now + 0.5


def wait_for_armed(master, snapshot):
    print("[arm] waiting for actual armed heartbeat")
    while True:
        msg = master.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue

        update_snapshot(snapshot, msg)
        print("[arm] armed=%s" % snapshot.armed)

        if snapshot.armed:
            return


def arm_if_requested(master, snapshot, allow_script_arm):
    if snapshot.armed:
        return
    if not allow_script_arm:
        raise RuntimeError(
            "CH6/CH7 allow control, but actual armed heartbeat is false. "
            "Use CH7 to arm first, or pass --allow-script-arm for bench-only testing."
        )
    print("[arm] script arming enabled; sending arm command")
    master.arducopter_arm()


def send_takeoff(master, altitude_m):
    print("[takeoff] target_altitude=%.2fm" % altitude_m)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        altitude_m,
    )


def send_land(master):
    print("[land] sending LAND mode")
    mode_id = master.mode_mapping().get("LAND")
    if mode_id is not None:
        master.set_mode(mode_id)
    else:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0,
        )


def mavlink_const(name, fallback):
    return getattr(mavutil.mavlink, name, fallback)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def send_body_velocity_target(master, forward_mps, right_mps, down_mps):
    type_mask = (
        mavlink_const("POSITION_TARGET_TYPEMASK_X_IGNORE", 1)
        | mavlink_const("POSITION_TARGET_TYPEMASK_Y_IGNORE", 2)
        | mavlink_const("POSITION_TARGET_TYPEMASK_Z_IGNORE", 4)
        | mavlink_const("POSITION_TARGET_TYPEMASK_AX_IGNORE", 64)
        | mavlink_const("POSITION_TARGET_TYPEMASK_AY_IGNORE", 128)
        | mavlink_const("POSITION_TARGET_TYPEMASK_AZ_IGNORE", 256)
        | mavlink_const("POSITION_TARGET_TYPEMASK_YAW_IGNORE", 1024)
    )

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavlink_const("MAV_FRAME_BODY_NED", 8),
        type_mask,
        0,
        0,
        0,
        forward_mps,
        right_mps,
        down_mps,
        0,
        0,
        0,
        0,
        0,
    )


def send_body_offset_position(master, forward_m, right_m, down_m):
    print(
        "[position-target] body_offset forward=%.2fm right=%.2fm down=%.2fm"
        % (forward_m, right_m, down_m)
    )
    type_mask = (
        mavlink_const("POSITION_TARGET_TYPEMASK_VX_IGNORE", 8)
        | mavlink_const("POSITION_TARGET_TYPEMASK_VY_IGNORE", 16)
        | mavlink_const("POSITION_TARGET_TYPEMASK_VZ_IGNORE", 32)
        | mavlink_const("POSITION_TARGET_TYPEMASK_AX_IGNORE", 64)
        | mavlink_const("POSITION_TARGET_TYPEMASK_AY_IGNORE", 128)
        | mavlink_const("POSITION_TARGET_TYPEMASK_AZ_IGNORE", 256)
        | mavlink_const("POSITION_TARGET_TYPEMASK_YAW_IGNORE", 1024)
        | mavlink_const("POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE", 2048)
    )
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavlink_const("MAV_FRAME_BODY_OFFSET_NED", 9),
        type_mask,
        forward_m,
        right_m,
        down_m,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def get_altitude(snapshot):
    return snapshot.range_m


def local_position_ready(snapshot):
    return (
        snapshot.local_x_m is not None
        and snapshot.local_y_m is not None
        and snapshot.yaw_rad is not None
    )


def yaw_ready(snapshot):
    return snapshot.yaw_rad is not None


def read_control_message(master, snapshot, blocking=True, timeout=0.1):
    message_types = [
        "HEARTBEAT",
        "RC_CHANNELS",
        "RANGEFINDER",
        "GLOBAL_POSITION_INT",
        "LOCAL_POSITION_NED",
        "ATTITUDE",
        "STATUSTEXT",
    ]
    msg = master.recv_match(
        type=message_types,
        blocking=blocking,
        timeout=timeout,
    )
    latest_msg = msg
    if msg is not None:
        update_snapshot(snapshot, msg)

    while True:
        msg = master.recv_match(type=message_types, blocking=False)
        if msg is None:
            break
        update_snapshot(snapshot, msg)
        latest_msg = msg

    return latest_msg


def wait_for_yaw_reference(master, snapshot, threshold):
    print("[yaw] waiting for ATTITUDE after autonomous control is allowed")
    next_print = 0.0

    while True:
        read_control_message(master, snapshot, blocking=True, timeout=0.1)
        ensure_still_allowed(snapshot, threshold)
        if yaw_ready(snapshot):
            print("[yaw] locked target yaw=%.1fdeg" % math.degrees(snapshot.yaw_rad))
            return snapshot.yaw_rad

        now = time.monotonic()
        if now >= next_print:
            print_snapshot(snapshot)
            next_print = now + 0.5


def wait_for_navigation_reference(master, snapshot):
    print("[nav] waiting for LOCAL_POSITION_NED + ATTITUDE")
    next_print = 0.0

    while True:
        read_control_message(master, snapshot, blocking=True, timeout=0.1)
        if local_position_ready(snapshot):
            print(
                "[nav] local_position x=%.2fm y=%.2fm yaw=%.1fdeg"
                % (
                    snapshot.local_x_m,
                    snapshot.local_y_m,
                    math.degrees(snapshot.yaw_rad),
                )
            )
            return

        now = time.monotonic()
        if now >= next_print:
            print_snapshot(snapshot)
            next_print = now + 0.5


def altitude_down_velocity(snapshot, target_altitude_m):
    altitude = get_altitude(snapshot)
    if altitude is None:
        return 0.0

    altitude_error_m = target_altitude_m - altitude
    return clamp(
        -ALTITUDE_HOLD_GAIN * altitude_error_m,
        -DEFAULT_MAX_VERTICAL_SPEED_MPS,
        DEFAULT_MAX_VERTICAL_SPEED_MPS,
    )


def verify_flight_limits(snapshot, target_altitude_m, target_yaw_rad, check_altitude=True):
    if snapshot.roll_deg is not None and abs(snapshot.roll_deg) > DEFAULT_MAX_ROLL_PITCH_DEG:
        raise RuntimeError("Roll angle exceeded safety limit: %.1fdeg" % snapshot.roll_deg)
    if snapshot.pitch_deg is not None and abs(snapshot.pitch_deg) > DEFAULT_MAX_ROLL_PITCH_DEG:
        raise RuntimeError("Pitch angle exceeded safety limit: %.1fdeg" % snapshot.pitch_deg)

    altitude = get_altitude(snapshot)
    if check_altitude and altitude is not None:
        altitude_error_m = target_altitude_m - altitude
        if abs(altitude_error_m) > DEFAULT_MAX_ALTITUDE_ERROR_M:
            raise RuntimeError("Altitude error exceeded safety limit: %.2fm" % altitude_error_m)

    if snapshot.yaw_rad is not None and target_yaw_rad is not None:
        yaw_error_deg = math.degrees(wrap_pi(snapshot.yaw_rad - target_yaw_rad))
        if abs(yaw_error_deg) > DEFAULT_MAX_YAW_ERROR_DEG:
            raise RuntimeError("Yaw error exceeded safety limit: %.1fdeg" % yaw_error_deg)


def hold_guided_velocity(master, snapshot, duration_s, target_altitude_m, target_yaw_rad, threshold, label):
    print("[hold] %s %.1fs" % (label, duration_s))
    deadline = time.monotonic() + duration_s
    next_command = 0.0
    next_print = 0.0
    command_period_s = 1.0 / FORWARD_COMMAND_RATE_HZ

    while time.monotonic() < deadline:
        read_control_message(master, snapshot, blocking=True, timeout=0.05)
        ensure_still_allowed(snapshot, threshold)
        verify_flight_limits(snapshot, target_altitude_m, target_yaw_rad)

        now = time.monotonic()
        if now >= next_command:
            send_body_velocity_target(
                master,
                0.0,
                0.0,
                altitude_down_velocity(snapshot, target_altitude_m),
            )
            next_command = now + command_period_s

        if now >= next_print:
            print_snapshot(snapshot)
            next_print = now + 0.5


def send_altitude_target(master, snapshot, target_altitude_m):
    altitude = get_altitude(snapshot)
    if altitude is not None and altitude > target_altitude_m + ALTITUDE_REACHED_TOLERANCE_M:
        print(
            "[altitude-target] current=%.2fm target=%.2fm; active descent control"
            % (altitude, target_altitude_m)
        )
        return

    send_takeoff(master, target_altitude_m)


def wait_until_altitude(master, snapshot, target_altitude_m, target_yaw_rad, threshold):
    print("[takeoff] active altitude control")
    next_print = 0.0
    next_command = 0.0
    stable_since = None
    command_period_s = 1.0 / FORWARD_COMMAND_RATE_HZ
    timeout_s = max(
        DEFAULT_MIN_ALTITUDE_TIMEOUT_S,
        target_altitude_m / DEFAULT_MAX_VERTICAL_SPEED_MPS * 3.0 + 5.0,
    )
    deadline = time.monotonic() + timeout_s

    while True:
        read_control_message(master, snapshot, blocking=True, timeout=0.05)
        ensure_still_allowed(snapshot, threshold)
        verify_flight_limits(snapshot, target_altitude_m, target_yaw_rad, check_altitude=False)

        altitude = get_altitude(snapshot)
        now = time.monotonic()
        if now >= deadline:
            raise RuntimeError("Altitude target timed out before reaching %.2fm" % target_altitude_m)

        if altitude is None:
            if now >= next_print:
                print("[takeoff] waiting for rangefinder height")
                print_snapshot(snapshot)
                next_print = now + 0.5
            continue

        error_m = target_altitude_m - altitude
        down_mps = altitude_down_velocity(snapshot, target_altitude_m)

        if now >= next_command:
            send_body_velocity_target(
                master,
                0.0,
                0.0,
                down_mps,
            )
            next_command = now + command_period_s

        if abs(error_m) <= ALTITUDE_REACHED_TOLERANCE_M:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= ALTITUDE_STABLE_SECONDS:
                print("[takeoff] altitude target reached")
                send_body_velocity_target(
                    master,
                    0.0,
                    0.0,
                    altitude_down_velocity(snapshot, target_altitude_m),
                )
                return
        else:
            stable_since = None

        if now >= next_print:
            print(
                "[takeoff] target=%.2fm altitude=%.2fm error=%.2fm down_v=%.2fm/s"
                % (target_altitude_m, altitude, error_m, down_mps)
            )
            next_print = now + 0.5


def wait_until_forward_position(
    master,
    snapshot,
    start_x_m,
    start_y_m,
    start_yaw_rad,
    forward_distance_m,
    speed_mps,
    target_altitude_m,
    threshold,
):
    print("[forward] continuous velocity control")
    stable_since = None
    next_print = 0.0
    next_command = 0.0
    command_period_s = 1.0 / FORWARD_COMMAND_RATE_HZ
    timeout_s = max(DEFAULT_MIN_FORWARD_TIMEOUT_S, forward_distance_m / speed_mps * 3.0 + 5.0)
    deadline = time.monotonic() + timeout_s
    forward_north = math.cos(start_yaw_rad)
    forward_east = math.sin(start_yaw_rad)
    right_north = -math.sin(start_yaw_rad)
    right_east = math.cos(start_yaw_rad)

    while True:
        read_control_message(master, snapshot, blocking=True, timeout=0.05)
        ensure_still_allowed(snapshot, threshold)
        verify_flight_limits(snapshot, target_altitude_m, start_yaw_rad)

        if time.monotonic() >= deadline:
            raise RuntimeError("Forward motion timed out before reaching target")

        if snapshot.local_x_m is None or snapshot.local_y_m is None:
            continue

        dx_m = snapshot.local_x_m - start_x_m
        dy_m = snapshot.local_y_m - start_y_m
        progress_m = dx_m * forward_north + dy_m * forward_east
        crosstrack_m = dx_m * right_north + dy_m * right_east
        remaining_m = forward_distance_m - progress_m
        now = time.monotonic()

        if abs(crosstrack_m) > DEFAULT_MAX_CROSSTRACK_ERROR_M:
            raise RuntimeError("Crosstrack error exceeded safety limit: %.2fm" % crosstrack_m)

        if now >= next_command:
            if remaining_m <= POSITION_REACHED_TOLERANCE_M:
                forward_mps = 0.0
            else:
                forward_mps = clamp(remaining_m * 0.6, 0.0, speed_mps)

            right_mps = clamp(
                -CROSSTRACK_GAIN * crosstrack_m,
                -DEFAULT_MAX_LATERAL_SPEED_MPS,
                DEFAULT_MAX_LATERAL_SPEED_MPS,
            )
            send_body_velocity_target(
                master,
                forward_mps,
                right_mps,
                altitude_down_velocity(snapshot, target_altitude_m),
            )
            next_command = now + command_period_s

        if remaining_m <= POSITION_REACHED_TOLERANCE_M and abs(crosstrack_m) <= POSITION_REACHED_TOLERANCE_M:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= POSITION_STABLE_SECONDS:
                print("[forward] position target reached")
                send_body_velocity_target(
                    master,
                    0.0,
                    0.0,
                    altitude_down_velocity(snapshot, target_altitude_m),
                )
                return
        else:
            stable_since = None

        if now >= next_print:
            print(
                "[forward] target=%.2fm progress=%.2fm remaining=%.2fm crosstrack=%.2fm"
                % (forward_distance_m, progress_m, remaining_m, crosstrack_m)
            )
            print_snapshot(snapshot)
            next_print = now + 0.5


def ensure_still_allowed(snapshot, threshold):
    if not control_allowed(snapshot, threshold):
        raise RuntimeError("Control gate lost: mode/CH6/CH7 no longer allow autonomous flight")


def run(args):
    if args.altitude <= 0:
        raise SystemExit("--altitude must be positive")
    if args.distance < 0:
        raise SystemExit("--distance must be non-negative")
    if args.speed <= 0:
        raise SystemExit("--speed must be positive")

    forward_distance_m = args.distance

    print("[safety] run only after propellers are installed correctly and the test area is clear")

    master = connect(DEFAULT_DEVICE, DEFAULT_BAUD, DEFAULT_HEARTBEAT_TIMEOUT_S)
    request_basic_streams(master)
    snapshot = Snapshot()

    wait_for_control_allowed(master, snapshot, DEFAULT_START_THRESHOLD)
    arm_if_requested(master, snapshot, False)
    wait_for_armed(master, snapshot)
    ensure_still_allowed(snapshot, DEFAULT_START_THRESHOLD)
    target_yaw_rad = wait_for_yaw_reference(master, snapshot, DEFAULT_START_THRESHOLD)

    in_flight = False
    try:
        send_altitude_target(master, snapshot, args.altitude)
        in_flight = True

        wait_until_altitude(
            master,
            snapshot,
            args.altitude,
            target_yaw_rad,
            DEFAULT_START_THRESHOLD,
        )
        ensure_still_allowed(snapshot, DEFAULT_START_THRESHOLD)

        wait_for_navigation_reference(master, snapshot)
        hold_guided_velocity(
            master,
            snapshot,
            DEFAULT_PRE_HOLD_S,
            args.altitude,
            target_yaw_rad,
            DEFAULT_START_THRESHOLD,
            "pre-forward hold",
        )
        start_x_m = snapshot.local_x_m
        start_y_m = snapshot.local_y_m
        start_yaw_rad = target_yaw_rad

        print("[forward] distance=%.2fm speed=%.2fm/s" % (
            forward_distance_m,
            args.speed,
        ))
        wait_until_forward_position(
            master,
            snapshot,
            start_x_m,
            start_y_m,
            start_yaw_rad,
            forward_distance_m,
            args.speed,
            args.altitude,
            DEFAULT_START_THRESHOLD,
        )

        hold_guided_velocity(
            master,
            snapshot,
            DEFAULT_POST_HOLD_S,
            args.altitude,
            start_yaw_rad,
            DEFAULT_START_THRESHOLD,
            "post-forward hold",
        )

        send_land(master)
        print("[land] monitoring for %.1fs" % DEFAULT_LAND_MONITOR_S)
        read_messages(master, snapshot, DEFAULT_LAND_MONITOR_S, print_period_s=1.0)

    except KeyboardInterrupt:
        print("\n[abort] interrupted by user")
        if in_flight:
            send_land(master)
    except Exception as exc:
        print("[abort] %s" % exc)
        if in_flight:
            send_land(master)
        raise


def parse_args():
    parser = argparse.ArgumentParser(
        description="GUIDED autonomous test: reach target altitude, fly to a forward target point, then land."
    )
    parser.add_argument("--altitude", type=float, default=0.5)
    parser.add_argument("--distance", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=0.15)
    return parser.parse_args()


def main():
    run(parse_args())


if __name__ == "__main__":
    main()

