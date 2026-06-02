from __future__ import annotations

import argparse
import asyncio
import contextlib
import os
from dataclasses import dataclass
from typing import Optional

from mavsdk import System


DEFAULT_SYSTEM_ADDRESS = os.environ.get(
    "PIXHAWK_SYSTEM_ADDRESS",
    "serial:///dev/ttyTHS1:921600",
)


@dataclass
class TelemetrySnapshot:
    flight_mode: Optional[str] = None
    armed: Optional[bool] = None
    in_air: Optional[bool] = None
    battery_percent: Optional[float] = None
    is_armable: Optional[bool] = None
    local_position_ok: Optional[bool] = None
    global_position_ok: Optional[bool] = None
    home_position_ok: Optional[bool] = None


def _enum_name(value: object) -> str:
    if hasattr(value, "name"):
        return str(getattr(value, "name"))
    return str(value)


async def _wait_for_first(async_iter, timeout_s: float):
    async def _inner():
        async for item in async_iter:
            return item
        raise RuntimeError("Stream closed before receiving any data")

    return await asyncio.wait_for(_inner(), timeout=timeout_s)


async def _watch_flight_mode(drone: System, snapshot: TelemetrySnapshot) -> None:
    async for flight_mode in drone.telemetry.flight_mode():
        snapshot.flight_mode = _enum_name(flight_mode)


async def _watch_armed(drone: System, snapshot: TelemetrySnapshot) -> None:
    async for armed in drone.telemetry.armed():
        snapshot.armed = bool(armed)


async def _watch_in_air(drone: System, snapshot: TelemetrySnapshot) -> None:
    async for in_air in drone.telemetry.in_air():
        snapshot.in_air = bool(in_air)


async def _watch_battery(drone: System, snapshot: TelemetrySnapshot) -> None:
    async for battery in drone.telemetry.battery():
        snapshot.battery_percent = float(battery.remaining_percent) * 100.0


async def _watch_health(drone: System, snapshot: TelemetrySnapshot) -> None:
    async for health in drone.telemetry.health():
        snapshot.is_armable = bool(getattr(health, "is_armable", False))
        snapshot.local_position_ok = bool(getattr(health, "is_local_position_ok", False))
        snapshot.global_position_ok = bool(getattr(health, "is_global_position_ok", False))
        snapshot.home_position_ok = bool(getattr(health, "is_home_position_ok", False))


async def _watch_status_text(drone: System) -> None:
    async for status_text in drone.telemetry.status_text():
        text_type = _enum_name(getattr(status_text, "type", "STATUS"))
        text = getattr(status_text, "text", "")
        print(f"[px4][{text_type}] {text}")


def _format_bool(value: Optional[bool]) -> str:
    if value is None:
        return "--"
    return "yes" if value else "no"


def _format_percent(value: Optional[float]) -> str:
    if value is None:
        return "--"
    return f"{value:.1f}%"


async def _print_summary(snapshot: TelemetrySnapshot, period_s: float) -> None:
    while True:
        print(
            "[summary] "
            f"mode={snapshot.flight_mode or '--'} "
            f"armed={_format_bool(snapshot.armed)} "
            f"in_air={_format_bool(snapshot.in_air)} "
            f"battery={_format_percent(snapshot.battery_percent)} "
            f"armable={_format_bool(snapshot.is_armable)} "
            f"local_pos={_format_bool(snapshot.local_position_ok)} "
            f"global_pos={_format_bool(snapshot.global_position_ok)} "
            f"home_pos={_format_bool(snapshot.home_position_ok)}"
        )
        await asyncio.sleep(period_s)


async def run(system_address: str, connect_timeout_s: float, summary_period_s: float) -> None:
    print(f"[connect] system_address={system_address}")
    drone = System()
    await drone.connect(system_address=system_address)

    print("[connect] waiting for Pixhawk heartbeat...")
    connection_state = await _wait_for_first(
        drone.core.connection_state(),
        timeout_s=connect_timeout_s,
    )

    if not getattr(connection_state, "is_connected", False):
        raise RuntimeError("Connected call returned, but no active system connection was reported")

    uuid = getattr(connection_state, "uuid", None)
    if uuid is None:
        print("[connect] connected to Pixhawk")
    else:
        print(f"[connect] connected to Pixhawk, uuid={uuid}")

    snapshot = TelemetrySnapshot()

    watchers = [
        asyncio.create_task(_watch_flight_mode(drone, snapshot)),
        asyncio.create_task(_watch_armed(drone, snapshot)),
        asyncio.create_task(_watch_in_air(drone, snapshot)),
        asyncio.create_task(_watch_battery(drone, snapshot)),
        asyncio.create_task(_watch_health(drone, snapshot)),
        asyncio.create_task(_watch_status_text(drone)),
    ]

    summary_task = asyncio.create_task(_print_summary(snapshot, summary_period_s))

    try:
        await asyncio.gather(*watchers, summary_task)
    except asyncio.CancelledError:
        raise
    finally:
        for task in watchers:
            task.cancel()
        summary_task.cancel()
        for task in watchers + [summary_task]:
            with contextlib.suppress(asyncio.CancelledError):
                await task


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phase 1: connect Jetson Nano to Pixhawk and print telemetry."
    )
    parser.add_argument(
        "--system-address",
        default=DEFAULT_SYSTEM_ADDRESS,
        help=(
            "MAVSDK system address, e.g. "
            "'serial:///dev/ttyTHS1:921600' or 'udpin://0.0.0.0:14540'"
        ),
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for the first heartbeat/connection state.",
    )
    parser.add_argument(
        "--summary-period",
        type=float,
        default=1.0,
        help="Seconds between summary prints.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(
            run(
                system_address=args.system_address,
                connect_timeout_s=args.connect_timeout,
                summary_period_s=args.summary_period,
            )
        )
    except KeyboardInterrupt:
        print("\n[exit] interrupted by user")


if __name__ == "__main__":
    main()

