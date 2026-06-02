"""Manual takeoff -> guided gate align / hold / optional commit-through."""

import argparse
import math
import time
from pathlib import Path

import numpy as np
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

try:
    from gate_pose import ArucoGatePoseProvider, GatePose
    from aruco_pose import open_capture
    GATE_POSE_IMPORT_ERROR = None
except ImportError as exc:
    ArucoGatePoseProvider = None
    open_capture = None
    GATE_POSE_IMPORT_ERROR = exc

    class GatePose:
        def __init__(
            self,
            valid,
            x_body_m,
            y_body_m,
            z_body_m,
            yaw_error_rad,
            source,
            layout,
            markers_seen,
        ):
            self.valid = valid
            self.x_body_m = x_body_m
            self.y_body_m = y_body_m
            self.z_body_m = z_body_m
            self.yaw_error_rad = yaw_error_rad
            self.source = source
            self.layout = layout
            self.markers_seen = markers_seen


DEFAULT_DEVICE = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600
GUIDED_MODE = "GUIDED"
ALIGN_RISE_TRIGGER_CHANNEL = 8


class FlightSnapshot:
    def __init__(self):
        self.mode = "--"
        self.armed = False
        self.ch6 = 0
        self.ch7 = 0
        self.ch8 = 0
        self.range_m = None
        self.range_source = "--"
        self.rel_alt_m = None
        self.local_x_m = None
        self.local_y_m = None
        self.local_z_m = None
        self.roll_deg = None
        self.pitch_deg = None
        self.yaw_deg = None
        self.status_text = ""


class SlewRateLimiter:
    def __init__(self, rate_per_s):
        self.rate_per_s = float(rate_per_s)
        self.value = 0.0
        self.initialized = False

    def reset(self, value=0.0):
        self.value = float(value)
        self.initialized = True

    def update(self, target, dt):
        target = float(target)
        dt = max(0.0, float(dt))
        if not self.initialized:
            self.reset(target)
            return self.value

        max_step = self.rate_per_s * dt
        delta = target - self.value
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        self.value += delta
        return self.value


class AxisController:
    def __init__(self, kp, deadband, limit, slew_rate):
        self.kp = float(kp)
        self.deadband = float(deadband)
        self.limit = abs(float(limit))
        self.slew = SlewRateLimiter(slew_rate)

    def reset(self):
        self.slew.reset(0.0)

    def _clip(self, value):
        if value > self.limit:
            return self.limit
        if value < -self.limit:
            return -self.limit
        return value

    def update(self, error, dt):
        error = float(error)
        if abs(error) < self.deadband:
            raw = 0.0
        else:
            raw = self.kp * error
        limited = self._clip(raw)
        return self.slew.update(limited, dt)


class MockPosePlant:
    def __init__(self, scenario):
        self.scenario = str(scenario)
        self.visible_after_s = 0.4
        self.loss_windows = []
        self.hide_when_x_below_m = None
        if self.scenario == "align_yz_yaw":
            self.x_body_m = 2.0
            self.y_body_m = 0.55
            self.z_body_m = -0.28
            self.yaw_error_rad = math.radians(18.0)
        elif self.scenario == "commit_through":
            self.x_body_m = 0.95
            self.y_body_m = 0.14
            self.z_body_m = -0.06
            self.yaw_error_rad = math.radians(4.0)
            self.hide_when_x_below_m = 0.65
        elif self.scenario == "loss_recovery":
            self.x_body_m = 2.0
            self.y_body_m = 0.45
            self.z_body_m = -0.22
            self.yaw_error_rad = math.radians(14.0)
            self.loss_windows = [(2.2, 2.8)]
        else:
            self.x_body_m = 3.2
            self.y_body_m = 0.55
            self.z_body_m = -0.28
            self.yaw_error_rad = math.radians(18.0)

    def step(self, command, dt):
        self.x_body_m = max(0.2, self.x_body_m - float(command["vx"]) * dt)
        self.y_body_m -= float(command["vy"]) * dt
        self.z_body_m -= float(command["vz"]) * dt
        self.yaw_error_rad -= float(command["yaw_rate"]) * dt

    def pose(self, elapsed_s):
        elapsed_s = float(elapsed_s)
        if elapsed_s < self.visible_after_s:
            return GatePose(False, 0.0, 0.0, 0.0, 0.0, "mock", "single", 0)
        if self.hide_when_x_below_m is not None and self.x_body_m <= self.hide_when_x_below_m:
            return GatePose(False, 0.0, 0.0, 0.0, 0.0, "mock", "single", 0)
        for start_s, end_s in self.loss_windows:
            if start_s <= elapsed_s <= end_s:
                return GatePose(False, 0.0, 0.0, 0.0, 0.0, "mock", "single", 0)
        return GatePose(
            True,
            float(self.x_body_m),
            float(self.y_body_m),
            float(self.z_body_m),
            float(self.yaw_error_rad),
            "mock",
            "single",
            1,
        )


def default_mode_axes(mode):
    mode = str(mode)
    if mode == "safe_only":
        return {"x": False, "y": False, "z": False, "yaw": False}
    if mode == "align_yz_yaw":
        return {"x": False, "y": True, "z": True, "yaw": True}
    if mode == "align_rise_commit":
        return {"x": False, "y": True, "z": False, "yaw": True}
    if mode == "hold_2m" or mode == "align_commit":
        return {"x": True, "y": True, "z": True, "yaw": True}
    raise ValueError("Unknown mode: %s" % mode)


def resolve_enabled_axes(args):
    defaults = default_mode_axes(args.mode)
    enabled = {}
    for axis_name, default_value in defaults.items():
        override = getattr(args, "enable_%s" % axis_name)
        enabled[axis_name] = default_value if override is None else bool(override)
    return enabled


def axis_signs(args):
    return {
        "x": int(args.sign_x),
        "y": int(args.sign_y),
        "z": int(args.sign_z),
        "yaw": int(args.sign_yaw),
    }


def make_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
    return {
        "vx": float(vx),
        "vy": float(vy),
        "vz": float(vz),
        "yaw_rate": float(yaw_rate),
    }


def parse_args():
    root = Path(__file__).resolve().parents[1]
    p = argparse.ArgumentParser(
        description="Manual takeoff -> gate align and hold using GatePose and MAVLink body velocities."
    )
    p.add_argument("--device", default=DEFAULT_DEVICE)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--layout", type=str, default=str(root / "config" / "gate_layout.yaml"))
    p.add_argument("--params", type=str, default=str(root / "camera_params.npz"))
    p.add_argument("--camera-id", type=int, default=0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--print-interval", type=float, default=0.5)
    p.add_argument("--loop-rate", type=float, default=10.0)
    p.add_argument("--start-threshold", type=int, default=1700)
    p.add_argument("--min-altitude", type=float, default=0.35)
    p.add_argument(
        "--mode",
        choices=["safe_only", "align_yz_yaw", "hold_2m", "align_commit", "align_rise_commit"],
        default="hold_2m",
        help="safe_only=M1, align_yz_yaw=M2, hold_2m=M3 baseline, align_commit=M5 skeleton, align_rise_commit=M6 two-stage rise-through",
    )
    p.add_argument(
        "--pose-source",
        choices=["live", "mock"],
        default="live",
        help="Use live camera + MAVLink or local mock pose replay.",
    )
    p.add_argument(
        "--mock-scenario",
        choices=["hold_2m", "align_yz_yaw", "loss_recovery", "commit_through"],
        default="hold_2m",
    )
    p.add_argument("--mock-duration", type=float, default=6.0)
    p.add_argument("--target-x-m", type=float, default=2.0)
    p.add_argument("--target-y-m", type=float, default=0.0)
    p.add_argument("--target-z-m", type=float, default=0.0)
    p.add_argument("--hold-seconds", type=float, default=1.5)
    p.add_argument(
        "--commit-seconds",
        type=float,
        default=1.0,
        help="For align_commit / align_rise_commit: fixed forward commit duration after alignment is stable.",
    )
    p.add_argument(
        "--commit-forward-speed",
        type=float,
        default=0.20,
        help="For align_commit / align_rise_commit: fixed forward body-x speed during COMMIT.",
    )
    p.add_argument(
        "--commit-blind-seconds",
        type=float,
        default=0.35,
        help="For align_commit only: keep fixed forward COMMIT for this long after the marker first disappears.",
    )
    p.add_argument(
        "--commit-passed-x-m",
        type=float,
        default=0.60,
        help="For align_commit only: if marker is still visible and x falls below this, treat COMMIT as complete.",
    )
    p.add_argument(
        "--commit-abort-y",
        type=float,
        default=0.30,
        help="For align_commit only: abort COMMIT if |err_y| grows beyond this while marker is visible.",
    )
    p.add_argument(
        "--commit-abort-z",
        type=float,
        default=0.30,
        help="For align_commit only: abort COMMIT if |err_z| grows beyond this while marker is visible.",
    )
    p.add_argument(
        "--commit-abort-yaw-deg",
        type=float,
        default=18.0,
        help="For align_commit only: abort COMMIT if |err_yaw| grows beyond this while marker is visible.",
    )
    p.add_argument(
        "--rise-seconds",
        type=float,
        default=1.0,
        help="For align_rise_commit only: fixed rise duration after y/yaw alignment is stable.",
    )
    p.add_argument(
        "--rise-speed-up",
        type=float,
        default=0.35,
        help="For align_rise_commit only: upward body speed magnitude (m/s) during RISE.",
    )
    p.add_argument("--x-tol", type=float, default=0.18)
    p.add_argument("--y-tol", type=float, default=0.12)
    p.add_argument("--z-tol", type=float, default=0.12)
    p.add_argument("--yaw-tol-deg", type=float, default=8.0)
    p.add_argument("--lost-timeout", type=float, default=0.5)
    p.add_argument("--vx-max", type=float, default=0.20)
    p.add_argument("--vy-max", type=float, default=0.18)
    p.add_argument("--vz-max", type=float, default=0.16)
    p.add_argument("--yaw-rate-max-deg", type=float, default=12.0)
    p.add_argument("--kp-x", type=float, default=0.35)
    p.add_argument("--kp-y", type=float, default=0.45)
    p.add_argument("--kp-z", type=float, default=0.40)
    p.add_argument("--kp-yaw", type=float, default=1.20)
    p.add_argument("--deadband-x", type=float, default=0.08)
    p.add_argument("--deadband-y", type=float, default=0.05)
    p.add_argument("--deadband-z", type=float, default=0.05)
    p.add_argument("--deadband-yaw-deg", type=float, default=4.0)
    p.add_argument("--slew-vx", type=float, default=0.25)
    p.add_argument("--slew-vy", type=float, default=0.25)
    p.add_argument("--slew-vz", type=float, default=0.20)
    p.add_argument("--slew-yaw-deg", type=float, default=20.0)
    p.add_argument(
        "--require-stable-seen",
        type=float,
        default=0.30,
        help="Require continuous valid GatePose for this many seconds before ALIGN.",
    )
    p.add_argument(
        "--allow-disarmed-control",
        action="store_true",
        help="Allow control logic while HEARTBEAT armed=false. Bench only.",
    )
    p.add_argument(
        "--bench-force-control",
        action="store_true",
        help="Bench only: bypass GUIDED/armed/CH6/CH7 and altitude gates so debug commands are generated.",
    )
    p.add_argument(
        "--disable-land-on-exit",
        dest="land_on_exit",
        action="store_false",
        help="Do not switch to LAND when the script exits. By default live armed runs land on exit.",
    )
    p.set_defaults(land_on_exit=True)
    for axis_name in ("x", "y", "z", "yaw"):
        p.add_argument(
            "--enable-%s" % axis_name,
            dest="enable_%s" % axis_name,
            action="store_true",
            default=None,
            help="Force-enable %s control for this run." % axis_name,
        )
        p.add_argument(
            "--disable-%s" % axis_name,
            dest="enable_%s" % axis_name,
            action="store_false",
            help="Force-disable %s control for this run." % axis_name,
        )
        p.add_argument(
            "--sign-%s" % axis_name,
            type=int,
            choices=[-1, 1],
            default=1,
            help="Multiply %s control error by this sign before control." % axis_name,
        )
    return p.parse_args()


def connect(device, baud, timeout_s=15.0):
    if mavutil is None:
        raise ImportError("pymavlink required: python3 -m pip install --user pymavlink")
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


def request_basic_streams(master):
    request_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5)
    request_stream(master, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10)
    request_stream(master, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10)
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
        snapshot.ch8 = int(getattr(msg, "chan8_raw", 0))
    elif msg_type == "DISTANCE_SENSOR":
        current_cm = int(getattr(msg, "current_distance", 0))
        if current_cm > 0:
            snapshot.range_m = current_cm / 100.0
            snapshot.range_source = "DIST"
    elif msg_type == "GLOBAL_POSITION_INT":
        snapshot.rel_alt_m = float(getattr(msg, "relative_alt", 0)) / 1000.0
    elif msg_type == "LOCAL_POSITION_NED":
        snapshot.local_x_m = float(getattr(msg, "x", 0.0))
        snapshot.local_y_m = float(getattr(msg, "y", 0.0))
        snapshot.local_z_m = float(getattr(msg, "z", 0.0))
    elif msg_type == "ATTITUDE":
        snapshot.roll_deg = math.degrees(float(getattr(msg, "roll", 0.0)))
        snapshot.pitch_deg = math.degrees(float(getattr(msg, "pitch", 0.0)))
        snapshot.yaw_deg = math.degrees(float(getattr(msg, "yaw", 0.0)))
    elif msg_type == "STATUSTEXT":
        snapshot.status_text = getattr(msg, "text", "")
        print("[status] severity=%s text=%s" % (getattr(msg, "severity", "--"), snapshot.status_text))


def drain_messages(master, snapshot):
    msg = master.recv_match(
        type=[
            "HEARTBEAT",
            "RC_CHANNELS",
            "DISTANCE_SENSOR",
            "GLOBAL_POSITION_INT",
            "LOCAL_POSITION_NED",
            "ATTITUDE",
            "STATUSTEXT",
        ],
        blocking=False,
    )
    while msg is not None:
        update_snapshot(snapshot, msg)
        msg = master.recv_match(
            type=[
                "HEARTBEAT",
                "RC_CHANNELS",
                "DISTANCE_SENSOR",
                "GLOBAL_POSITION_INT",
                "LOCAL_POSITION_NED",
                "ATTITUDE",
                "STATUSTEXT",
            ],
            blocking=False,
        )


def get_altitude_m(snapshot):
    if snapshot.range_m is not None:
        return snapshot.range_m
    return snapshot.rel_alt_m


def print_snapshot(snapshot):
    altitude = get_altitude_m(snapshot)
    alt_text = "--" if altitude is None else "%.2fm[%s]" % (altitude, snapshot.range_source)
    rel_alt_text = "--" if snapshot.rel_alt_m is None else "%.2fm" % snapshot.rel_alt_m
    local_text = (
        "--"
        if (
            snapshot.local_x_m is None
            or snapshot.local_y_m is None
            or snapshot.local_z_m is None
        )
        else "x=%.2fm y=%.2fm z=%.2fm" % (
            snapshot.local_x_m,
            snapshot.local_y_m,
            snapshot.local_z_m,
        )
    )
    roll_text = "--" if snapshot.roll_deg is None else "%.1fdeg" % snapshot.roll_deg
    pitch_text = "--" if snapshot.pitch_deg is None else "%.1fdeg" % snapshot.pitch_deg
    yaw_text = "--" if snapshot.yaw_deg is None else "%.1fdeg" % snapshot.yaw_deg
    print(
        "[telemetry] mode=%s armed=%s CH6=%s CH7=%s CH8=%s range=%s rel_alt=%s local=%s roll=%s pitch=%s yaw=%s"
        % (
            snapshot.mode,
            snapshot.armed,
            snapshot.ch6,
            snapshot.ch7,
            snapshot.ch8,
            alt_text,
            rel_alt_text,
            local_text,
            roll_text,
            pitch_text,
            yaw_text,
        )
    )


def control_allowed(snapshot, threshold, require_armed):
    return (
        snapshot.mode == GUIDED_MODE
        and snapshot.ch6 > threshold
        and snapshot.ch7 > threshold
        and ((not require_armed) or snapshot.armed)
    )


def send_body_velocity(master, vx, vy, vz, yaw_rate=0.0):
    if master is None:
        return
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0,
        yaw_rate,
    )


def issue_command(master, command_state, vx, vy, vz, yaw_rate=0.0):
    command_state.update(make_command(vx, vy, vz, yaw_rate))
    send_body_velocity(master, vx, vy, vz, yaw_rate)


def zero_command(master, command_state, cycles=3, sleep_s=0.05):
    for _ in range(cycles):
        issue_command(master, command_state, 0.0, 0.0, 0.0, 0.0)
        time.sleep(sleep_s)


def send_land(master):
    if master is None:
        return
    print("[land] sending LAND mode")
    mode_id = None
    try:
        mode_mapping = master.mode_mapping()
        if mode_mapping is not None:
            mode_id = mode_mapping.get("LAND")
    except Exception:
        mode_id = None

    if mode_id is not None:
        master.set_mode(mode_id)
        return

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,
        0, 0, 0,
    )


def pose_errors(pose, args):
    return {
        "x": float(pose.x_body_m - args.target_x_m),
        "y": float(pose.y_body_m - args.target_y_m),
        "z": float(pose.z_body_m - args.target_z_m),
        "yaw": float(pose.yaw_error_rad),
    }


def pose_within_tolerance(pose, args, enabled_axes):
    errors = pose_errors(pose, args)
    checks = []
    if enabled_axes["x"]:
        checks.append(abs(errors["x"]) <= args.x_tol)
    if enabled_axes["y"]:
        checks.append(abs(errors["y"]) <= args.y_tol)
    if enabled_axes["z"]:
        checks.append(abs(errors["z"]) <= args.z_tol)
    if enabled_axes["yaw"]:
        checks.append(abs(math.degrees(errors["yaw"])) <= args.yaw_tol_deg)
    if not checks:
        return True
    return all(checks)


def commit_abort_required(errors, args, enabled_axes):
    if enabled_axes["y"] and abs(errors["y"]) > args.commit_abort_y:
        return True
    if enabled_axes["z"] and abs(errors["z"]) > args.commit_abort_z:
        return True
    if enabled_axes["yaw"] and abs(math.degrees(errors["yaw"])) > args.commit_abort_yaw_deg:
        return True
    return False


def build_controllers(args):
    return {
        "x": AxisController(args.kp_x, args.deadband_x, args.vx_max, args.slew_vx),
        "y": AxisController(args.kp_y, args.deadband_y, args.vy_max, args.slew_vy),
        "z": AxisController(args.kp_z, args.deadband_z, args.vz_max, args.slew_vz),
        "yaw": AxisController(
            args.kp_yaw,
            math.radians(args.deadband_yaw_deg),
            math.radians(args.yaw_rate_max_deg),
            math.radians(args.slew_yaw_deg),
        ),
    }


def reset_controllers(controllers):
    for controller in controllers.values():
        controller.reset()


def compute_commands(controllers, errors, enabled_axes, signs, dt):
    commands = make_command()
    axis_to_cmd_key = {"x": "vx", "y": "vy", "z": "vz", "yaw": "yaw_rate"}
    for axis_name, cmd_key in axis_to_cmd_key.items():
        if enabled_axes[axis_name]:
            commands[cmd_key] = controllers[axis_name].update(errors[axis_name] * signs[axis_name], dt)
        else:
            controllers[axis_name].reset()
            commands[cmd_key] = 0.0
    return commands


def active_axes_for_errors(mode, errors, enabled_axes, args):
    active_axes = dict(enabled_axes)
    stage_label = None
    if mode != "align_yz_yaw" or errors is None:
        return active_axes, stage_label

    if enabled_axes["z"] and abs(errors["z"]) > args.z_tol:
        active_axes = {"x": False, "y": False, "z": True, "yaw": False}
        stage_label = "stage=z"
    elif enabled_axes["yaw"] and abs(math.degrees(errors["yaw"])) > args.yaw_tol_deg:
        active_axes = {"x": False, "y": False, "z": False, "yaw": True}
        stage_label = "stage=yaw"
    elif enabled_axes["y"] and abs(errors["y"]) > args.y_tol:
        active_axes = {"x": False, "y": True, "z": False, "yaw": False}
        stage_label = "stage=y"
    return active_axes, stage_label


def enabled_axes_text(enabled_axes, signs):
    parts = []
    for axis_name in ("x", "y", "z", "yaw"):
        if enabled_axes[axis_name]:
            parts.append("%s(sign=%+d)" % (axis_name, signs[axis_name]))
        else:
            parts.append("%s(off)" % axis_name)
    return " ".join(parts)


def phase_label(state):
    mapping = {
        "SAFE": "safe_idle",
        "SEARCH": "search_target",
        "ALIGN": "aligning",
        "HOLD": "aligned_hold",
        "RISE": "rising",
        "COMMIT": "forward_commit",
        "DONE": "complete",
    }
    return mapping.get(state, str(state).lower())


def stage_label_for_output(state, mode, errors, enabled_axes, args):
    active_axes, stage_label = active_axes_for_errors(mode, errors, enabled_axes, args)
    if stage_label is not None:
        return active_axes, stage_label
    if mode != "align_yz_yaw":
        return active_axes, "stage=none"
    if state == "HOLD":
        return active_axes, "stage=hold"
    if state == "SEARCH":
        return active_axes, "stage=wait"
    return active_axes, "stage=none"


def print_debug(state, state_reason, mode, last_pose, seen_recently, command_state, errors, enabled_axes, stage_label=None):
    if mode == "safe_only":
        print("[debug] mode=safe_only state=%s reason=%s command=vx=0 vy=0 vz=0 yaw_rate=0" % (
            state,
            state_reason,
        ))
        return

    if last_pose is None or not seen_recently or errors is None:
        print("[debug] mode=%s state=%s reason=%s pose=stale cmd_vx=%.3f cmd_vy=%.3f cmd_vz=%.3f cmd_yaw=%.1fdeg/s" % (
            mode,
            state,
            state_reason,
            command_state["vx"],
            command_state["vy"],
            command_state["vz"],
            math.degrees(command_state["yaw_rate"]),
        ))
        return

    parts = ["[debug] mode=%s state=%s reason=%s" % (mode, state, state_reason)]
    if stage_label is not None:
        parts.append(stage_label)
    if enabled_axes["x"]:
        parts.append("err_x=%.3f" % errors["x"])
        parts.append("cmd_vx=%.3f" % command_state["vx"])
    if enabled_axes["y"]:
        parts.append("err_y=%.3f" % errors["y"])
        parts.append("cmd_vy=%.3f" % command_state["vy"])
    if enabled_axes["z"]:
        parts.append("err_z=%.3f" % errors["z"])
        parts.append("cmd_vz=%.3f" % command_state["vz"])
    if enabled_axes["yaw"]:
        parts.append("err_yaw=%.1fdeg" % math.degrees(errors["yaw"]))
        parts.append("cmd_yaw=%.1fdeg/s" % math.degrees(command_state["yaw_rate"]))
    print(" ".join(parts))


def make_mock_snapshot():
    snapshot = FlightSnapshot()
    snapshot.mode = GUIDED_MODE
    snapshot.armed = True
    snapshot.ch6 = 1900
    snapshot.ch7 = 1900
    snapshot.ch8 = 1000
    snapshot.range_m = 1.0
    snapshot.rel_alt_m = 1.0
    snapshot.roll_deg = 0.0
    snapshot.pitch_deg = 0.0
    snapshot.yaw_deg = 0.0
    return snapshot


def main():
    args = parse_args()
    enabled_axes = resolve_enabled_axes(args)
    signs = axis_signs(args)

    provider = None
    cap = None
    master = None
    if args.pose_source == "live":
        if GATE_POSE_IMPORT_ERROR is not None:
            raise ImportError(
                "live pose source requires gate_pose/aruco dependencies: %s"
                % GATE_POSE_IMPORT_ERROR
            )
        params_path = Path(args.params)
        if not params_path.exists():
            raise FileNotFoundError("Camera params not found: %s" % params_path)

        data = np.load(str(params_path))
        provider = ArucoGatePoseProvider(args.layout, data["camera_matrix"], data["dist_coeffs"])
        cap = open_capture(args.camera_id)
        cap.set(3, args.width)
        cap.set(4, args.height)
        if not cap.isOpened():
            raise RuntimeError("Cannot open camera id=%s" % args.camera_id)

        master = connect(args.device, args.baud)
        request_basic_streams(master)
        snapshot = FlightSnapshot()
        mock_plant = None
        mock_start = None
    else:
        snapshot = make_mock_snapshot()
        mock_plant = MockPosePlant(args.mock_scenario)
        mock_start = time.monotonic()
    controllers = build_controllers(args)
    reset_controllers(controllers)
    command_state = make_command()

    print("[info] mode=%s pose_source=%s" % (args.mode, args.pose_source))
    print("[info] enabled axes: %s" % enabled_axes_text(enabled_axes, signs))
    print("[info] target gate pose in body FRD: x=%.2fm y=%.2fm z=%.2fm yaw=0deg" % (
        args.target_x_m,
        args.target_y_m,
        args.target_z_m,
    ))
    print("[info] velocity limits: vx<=%.2f vy<=%.2f vz<=%.2f yaw_rate<=%.1fdeg/s" % (
        args.vx_max,
        args.vy_max,
        args.vz_max,
        args.yaw_rate_max_deg,
    ))
    if args.allow_disarmed_control:
        print("[warn] disarmed control allowed for bench testing only")
    if args.bench_force_control:
        print("[warn] bench force control active: bypassing GUIDED/armed/CH6/CH7 and altitude gates")
    if args.mode == "align_rise_commit":
        print("[info] align_rise_commit: ALIGN uses y+yaw only, then fixed RISE, then fixed forward COMMIT")
        print(
            "[info] align_rise_commit trigger: CH%s rising edge above %s starts one sequence"
            % (ALIGN_RISE_TRIGGER_CHANNEL, args.start_threshold)
        )
    if args.pose_source == "live":
        if args.bench_force_control:
            print("[info] live pose + bench force control: generating commands without GUIDED/armed/CH6/CH7 or altitude gates")
        else:
            print("[info] waiting for manual takeoff, GUIDED mode, armed heartbeat, and CH6/CH7 high")
    else:
        print("[info] mock pose replay active for %.1fs using scenario=%s" % (
            args.mock_duration,
            args.mock_scenario,
        ))

    state = "SEARCH"
    state_reason = "startup"
    stable_seen_since = None
    aligned_since = None
    rise_since = None
    commit_since = None
    commit_blind_since = None
    require_sequence_trigger = args.mode == "align_rise_commit" and args.pose_source == "live"
    trigger_seen_low = not require_sequence_trigger
    trigger_prev_high = False
    sequence_requested = not require_sequence_trigger
    last_valid_pose_time = 0.0
    last_pose = None
    last_print = 0.0
    last_loop = time.monotonic()
    period_s = 1.0 / max(1.0, args.loop_rate)
    exit_exception = None

    try:
        while True:
            loop_start = time.monotonic()
            dt = max(1e-3, loop_start - last_loop)
            last_loop = loop_start

            pose = None
            if args.pose_source == "live":
                drain_messages(master, snapshot)
                ok, frame = cap.read()
                if ok:
                    pose = provider.estimate(frame)
                    if pose.valid:
                        last_pose = pose
                        last_valid_pose_time = loop_start
            else:
                mock_plant.step(command_state, dt)
                pose = mock_plant.pose(loop_start - mock_start)
                if pose.valid:
                    last_pose = pose
                    last_valid_pose_time = loop_start

            raw_allowed = control_allowed(
                snapshot,
                args.start_threshold,
                require_armed=(not args.allow_disarmed_control),
            )
            altitude = get_altitude_m(snapshot)
            raw_altitude_ok = altitude is not None and altitude >= args.min_altitude
            allowed = True if args.bench_force_control else raw_allowed
            altitude_ok = True if args.bench_force_control else raw_altitude_ok
            seen_recently = (loop_start - last_valid_pose_time) <= args.lost_timeout
            trigger_high = snapshot.ch8 > args.start_threshold
            if not trigger_high:
                trigger_seen_low = True
            trigger_rising_edge = (
                require_sequence_trigger
                and trigger_seen_low
                and (not trigger_prev_high)
                and trigger_high
            )
            if trigger_rising_edge and state != "DONE":
                sequence_requested = True
                print("[trigger] CH%s rising edge detected; sequence armed" % ALIGN_RISE_TRIGGER_CHANNEL)
            trigger_prev_high = trigger_high

            current_errors = None
            if last_pose is not None and seen_recently:
                current_errors = pose_errors(last_pose, args)

            if args.mode == "safe_only":
                state = "SAFE" if (allowed and altitude_ok) else "SEARCH"
                if allowed and altitude_ok:
                    state_reason = "safe_only active; holding zero command"
                elif not allowed:
                    state_reason = "waiting control gate (GUIDED/armed/CH6/CH7)"
                else:
                    state_reason = "waiting minimum altitude"
                stable_seen_since = None
                aligned_since = None
                rise_since = None
                commit_since = None
                commit_blind_since = None
                reset_controllers(controllers)
                zero_command(master, command_state, cycles=1, sleep_s=0.0)
            elif not allowed or not altitude_ok:
                state = "SEARCH"
                if not allowed:
                    state_reason = "control gate closed (GUIDED/armed/CH6/CH7)"
                else:
                    state_reason = "below minimum altitude"
                stable_seen_since = None
                aligned_since = None
                rise_since = None
                commit_since = None
                commit_blind_since = None
                reset_controllers(controllers)
                zero_command(master, command_state, cycles=1, sleep_s=0.0)
            else:
                pose_valid_now = pose is not None and pose.valid
                if pose is not None and pose.valid:
                    if stable_seen_since is None:
                        stable_seen_since = loop_start
                else:
                    stable_seen_since = None

                if state == "SEARCH":
                    zero_command(master, command_state, cycles=1, sleep_s=0.0)
                    if require_sequence_trigger and not sequence_requested:
                        state_reason = "waiting for CH%s rising-edge trigger" % ALIGN_RISE_TRIGGER_CHANNEL
                    elif pose is None or not pose.valid:
                        if seen_recently and last_pose is not None:
                            state_reason = "waiting fresh pose; using recent stale pose only"
                        else:
                            state_reason = "waiting for visible marker"
                    elif stable_seen_since is None:
                        state_reason = "waiting for stable pose timer"
                    else:
                        stable_for_s = loop_start - stable_seen_since
                        if stable_for_s >= args.require_stable_seen:
                            state_reason = "stable pose acquired; entering ALIGN"
                        else:
                            state_reason = "waiting stable pose %.2f/%.2fs" % (
                                stable_for_s,
                                args.require_stable_seen,
                            )
                    ready_for_align = (
                        stable_seen_since is not None
                        and (loop_start - stable_seen_since) >= args.require_stable_seen
                    )
                    if ready_for_align and (sequence_requested or not require_sequence_trigger):
                        state = "ALIGN"
                        if require_sequence_trigger:
                            sequence_requested = False
                        aligned_since = None
                        rise_since = None
                        commit_since = None
                        commit_blind_since = None
                        reset_controllers(controllers)
                elif state == "ALIGN":
                    if not seen_recently or last_pose is None:
                        state = "SEARCH"
                        state_reason = "lost target; back to SEARCH"
                        stable_seen_since = None
                        aligned_since = None
                        rise_since = None
                        commit_since = None
                        commit_blind_since = None
                        reset_controllers(controllers)
                        zero_command(master, command_state, cycles=1, sleep_s=0.0)
                    else:
                        current_errors = pose_errors(last_pose, args)
                        commands = compute_commands(controllers, current_errors, enabled_axes, signs, dt)
                        issue_command(
                            master,
                            command_state,
                            commands["vx"],
                            commands["vy"],
                            commands["vz"],
                            commands["yaw_rate"],
                        )

                        if pose_within_tolerance(last_pose, args, enabled_axes):
                            if aligned_since is None:
                                aligned_since = loop_start
                                state_reason = "within tolerance; starting hold timer"
                            elif (loop_start - aligned_since) >= args.hold_seconds:
                                if args.mode == "align_commit":
                                    state = "COMMIT"
                                    state_reason = "hold timer reached; entering COMMIT"
                                    commit_since = loop_start
                                    commit_blind_since = None
                                elif args.mode == "align_rise_commit":
                                    state = "RISE"
                                    state_reason = "hold timer reached; entering RISE"
                                    rise_since = loop_start
                                    commit_since = None
                                    commit_blind_since = None
                                else:
                                    state = "HOLD"
                                    state_reason = "hold timer reached; entering HOLD"
                                reset_controllers(controllers)
                            else:
                                state_reason = "within tolerance %.2f/%.2fs" % (
                                    loop_start - aligned_since,
                                    args.hold_seconds,
                                )
                        else:
                            aligned_since = None
                            state_reason = "correcting pose error"
                elif state == "RISE":
                    elapsed_rise_s = 0.0 if rise_since is None else (loop_start - rise_since)
                    if elapsed_rise_s >= args.rise_seconds:
                        state = "COMMIT"
                        state_reason = "rise window complete; entering COMMIT"
                        commit_since = loop_start
                        commit_blind_since = None
                        reset_controllers(controllers)
                    else:
                        issue_command(
                            master,
                            command_state,
                            0.0,
                            0.0,
                            -min(abs(args.rise_speed_up), args.vz_max),
                            0.0,
                        )
                        state_reason = "rising %.2f/%.2fs" % (
                            elapsed_rise_s,
                            args.rise_seconds,
                        )
                elif state == "HOLD":
                    if not seen_recently or last_pose is None:
                        state = "SEARCH"
                        state_reason = "lost target during HOLD; back to SEARCH"
                        stable_seen_since = None
                        aligned_since = None
                        rise_since = None
                        commit_since = None
                        commit_blind_since = None
                        reset_controllers(controllers)
                        zero_command(master, command_state, cycles=1, sleep_s=0.0)
                    else:
                        current_errors = pose_errors(last_pose, args)
                        commands = compute_commands(controllers, current_errors, enabled_axes, signs, dt)
                        issue_command(
                            master,
                            command_state,
                            commands["vx"],
                            commands["vy"],
                            commands["vz"],
                            commands["yaw_rate"],
                        )

                        if not pose_within_tolerance(last_pose, args, enabled_axes):
                            state = "ALIGN"
                            state_reason = "left hold tolerance; back to ALIGN"
                            aligned_since = None
                        else:
                            state_reason = "holding aligned pose"
                elif state == "COMMIT":
                    elapsed_commit_s = 0.0 if commit_since is None else (loop_start - commit_since)
                    if elapsed_commit_s >= args.commit_seconds:
                        if args.mode == "align_rise_commit":
                            state = "DONE"
                            state_reason = "commit window complete; entering DONE"
                        else:
                            state = "SEARCH"
                            state_reason = "commit window complete; back to SEARCH"
                        stable_seen_since = None
                        aligned_since = None
                        rise_since = None
                        commit_since = None
                        commit_blind_since = None
                        reset_controllers(controllers)
                        zero_command(master, command_state, cycles=1, sleep_s=0.0)
                    elif args.mode == "align_rise_commit":
                        issue_command(
                            master,
                            command_state,
                            min(abs(args.commit_forward_speed), args.vx_max),
                            0.0,
                            0.0,
                            0.0,
                        )
                        state_reason = "committing forward %.2f/%.2fs" % (
                            elapsed_commit_s,
                            args.commit_seconds,
                        )
                    elif pose_valid_now and last_pose is not None:
                        current_errors = pose_errors(last_pose, args)
                        commit_blind_since = None
                        if (
                            args.commit_passed_x_m > 0.0
                            and last_pose.x_body_m <= args.commit_passed_x_m
                        ):
                            state = "SEARCH"
                            state_reason = "commit passed x threshold; back to SEARCH"
                            stable_seen_since = None
                            aligned_since = None
                            rise_since = None
                            commit_since = None
                            commit_blind_since = None
                            reset_controllers(controllers)
                            zero_command(master, command_state, cycles=1, sleep_s=0.0)
                        elif commit_abort_required(current_errors, args, enabled_axes):
                            state = "SEARCH"
                            state_reason = "commit alignment exceeded abort threshold; back to SEARCH"
                            stable_seen_since = None
                            aligned_since = None
                            rise_since = None
                            commit_since = None
                            commit_blind_since = None
                            reset_controllers(controllers)
                            zero_command(master, command_state, cycles=1, sleep_s=0.0)
                        else:
                            issue_command(
                                master,
                                command_state,
                                min(abs(args.commit_forward_speed), args.vx_max),
                                0.0,
                                0.0,
                                0.0,
                            )
                            state_reason = "committing forward %.2f/%.2fs" % (
                                elapsed_commit_s,
                                args.commit_seconds,
                            )
                    else:
                        if commit_blind_since is None:
                            commit_blind_since = loop_start
                        blind_for_s = loop_start - commit_blind_since
                        if blind_for_s > args.commit_blind_seconds:
                            state = "SEARCH"
                            state_reason = "commit blind timeout; aborting to SEARCH"
                            stable_seen_since = None
                            aligned_since = None
                            rise_since = None
                            commit_since = None
                            commit_blind_since = None
                            reset_controllers(controllers)
                            zero_command(master, command_state, cycles=1, sleep_s=0.0)
                        else:
                            issue_command(
                                master,
                                command_state,
                                min(abs(args.commit_forward_speed), args.vx_max),
                                0.0,
                                0.0,
                                0.0,
                            )
                            state_reason = "commit blind continue %.2f/%.2fs" % (
                                blind_for_s,
                                args.commit_blind_seconds,
                            )
                elif state == "DONE":
                    reset_controllers(controllers)
                    zero_command(master, command_state, cycles=1, sleep_s=0.0)
                    state_reason = "sequence complete; holding zero command"

            now = time.monotonic()
            if now - last_print >= args.print_interval:
                print_snapshot(snapshot)
                pose_text = "pose=invalid"
                if last_pose is not None and seen_recently:
                    pose_text = (
                        "pose=x=%.3f y=%.3f z=%.3f yaw=%.1fdeg"
                        % (
                            last_pose.x_body_m,
                            last_pose.y_body_m,
                            last_pose.z_body_m,
                            math.degrees(last_pose.yaw_error_rad),
                        )
                    )
                gate_text = "altitude_ok=%s allowed=%s" % (altitude_ok, allowed)
                if args.bench_force_control:
                    gate_text += " raw_altitude_ok=%s raw_allowed=%s" % (
                        raw_altitude_ok,
                        raw_allowed,
                    )
                active_axes, current_stage_label = stage_label_for_output(
                    state,
                    args.mode,
                    current_errors,
                    enabled_axes,
                    args,
                )
                print("[control] phase=%s %s state=%s reason=%s %s %s" % (
                    phase_label(state),
                    current_stage_label,
                    state,
                    state_reason,
                    gate_text,
                    pose_text,
                ))
                print_debug(
                    state,
                    state_reason,
                    args.mode,
                    last_pose,
                    seen_recently,
                    command_state,
                    current_errors,
                    active_axes,
                    current_stage_label,
                )
                last_print = now

            if args.pose_source == "mock" and (loop_start - mock_start) >= args.mock_duration:
                print("[stop] mock replay complete")
                break

            sleep_s = period_s - (time.monotonic() - loop_start)
            if sleep_s > 0.0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("\n[stop] interrupted by user")
    except Exception as exc:
        exit_exception = exc
        print("[abort] %s" % exc)
        raise
    finally:
        zero_command(master, command_state, cycles=5, sleep_s=0.05)
        should_land_on_exit = (
            args.land_on_exit
            and args.pose_source == "live"
            and (not args.bench_force_control)
            and master is not None
            and snapshot is not None
            and snapshot.armed
        )
        if should_land_on_exit:
            print("[land] exit handler requesting LAND")
            try:
                send_land(master)
            except Exception as land_exc:
                print("[land] failed to request LAND: %s" % land_exc)
        if cap is not None:
            cap.release()


if __name__ == "__main__":
    main()
