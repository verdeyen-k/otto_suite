#!/usr/bin/env python3
"""Web-based joystick + telemetry bridge for otto_suite's ZeroMQ link
(plan.md Layer 4) -- replaces teleop_joystick.py for normal operation.
Serves a webpage from this machine (run it on ottoman-control, alongside
teleop) that reads an Xbox controller via the browser's Gamepad API and
shows live per-module telemetry, so the machine with the physical
controller no longer needs pygame/pyzmq installed -- just a browser.

This process ONLY relays: browser <-WebSocket-> this bridge <-ZeroMQ->
teleop. It holds none of teleop's safety logic (stale-link watchdog, rate
limiting, acceleration limiting, wheel-speed desaturation) -- that is all
still in teleop.cpp, completely unmodified. If this process dies, a
browser tab is closed, or the tab is backgrounded (browsers throttle
requestAnimationFrame heavily when not focused), teleop simply stops
receiving commands and its own watchdog disables the actuators exactly as
it already does today for a dropped teleop_joystick.py.

Wire format must match src/bridge/messages.hpp exactly (little-endian,
packed, no padding):
  command:   3x double                                       = 24 bytes
  telemetry: 3x double + 1 byte + 4x(2x double + 1 byte)      = 93 bytes

Usage:
  pip install aiohttp pyzmq
  python3 bridge.py [--zmq-host HOST] [--http-port 8080] [--config PATH]

Then, from a browser on any machine that can reach this one (and has the
Xbox controller attached), open http://<this machine>:8080/ and press a
button on the controller to activate it.
"""
import argparse
import asyncio
import contextlib
import json
import struct
from pathlib import Path

import zmq
import zmq.asyncio
from aiohttp import WSMsgType, web

COMMAND_PORT = 5555
TELEMETRY_PORT = 5556
MODULE_NAMES = ("fl", "fr", "rl", "rr")

COMMAND_FORMAT = "<ddd"
TELEMETRY_FORMAT = "<dddB" + "ddB" * 4
assert struct.calcsize(COMMAND_FORMAT) == 24, "must match ChassisCommandWire in messages.hpp"
assert struct.calcsize(TELEMETRY_FORMAT) == 93, "must match ChassisTelemetryWire in messages.hpp"

STATIC_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = STATIC_DIR.parents[1] / "config" / "robot_constants.yaml"


def load_speed_limits(config_path):
    """Same flat 'key: value' parsing as robot_config.cpp -- reads just the
    two keys the web page needs to scale joystick axes the same way
    teleop's own defaults do."""
    values = {}
    with open(config_path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, _, value = line.partition(":")
            values[key.strip()] = value.strip()
    return {
        "max_speed_mps": float(values["max_speed_mps"]),
        "max_omega_deg_s": float(values["max_omega_deg_s"]),
    }


async def index(request):
    return web.FileResponse(STATIC_DIR / "index.html")


async def config_handler(request):
    return web.json_response(request.app["speed_limits"])


async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    zmq_ctx = request.app["zmq_ctx"]
    zmq_host = request.app["zmq_host"]

    command_pub = zmq_ctx.socket(zmq.PUB)
    command_pub.connect(f"tcp://{zmq_host}:{COMMAND_PORT}")
    telemetry_sub = zmq_ctx.socket(zmq.SUB)
    telemetry_sub.setsockopt(zmq.SUBSCRIBE, b"")
    telemetry_sub.setsockopt(zmq.CONFLATE, 1)
    telemetry_sub.connect(f"tcp://{zmq_host}:{TELEMETRY_PORT}")

    async def forward_commands():
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
                wire = struct.pack(COMMAND_FORMAT, data["vx"], data["vy"], data["omega"])
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
            command_pub.send(wire)

    async def forward_telemetry():
        while True:
            data = await telemetry_sub.recv()
            fields = struct.unpack(TELEMETRY_FORMAT, data)
            vx, vy, omega, any_fault = fields[0], fields[1], fields[2], fields[3]
            modules = []
            for i, name in enumerate(MODULE_NAMES):
                angle_deg, speed_mps, fault = fields[4 + i * 3 : 4 + i * 3 + 3]
                modules.append(
                    {"name": name, "angle_deg": angle_deg, "speed_mps": speed_mps, "fault": bool(fault)}
                )
            await ws.send_json(
                {"odom": {"vx": vx, "vy": vy, "omega": omega}, "any_fault": bool(any_fault), "modules": modules}
            )

    telemetry_task = asyncio.ensure_future(forward_telemetry())
    try:
        await forward_commands()
    finally:
        telemetry_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await telemetry_task
        command_pub.close()
        telemetry_sub.close()

    return ws


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--zmq-host", default="localhost", help="Host running teleop (usually this machine)")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="Path to robot_constants.yaml")
    args = parser.parse_args()

    app = web.Application()
    app["zmq_ctx"] = zmq.asyncio.Context()
    app["zmq_host"] = args.zmq_host
    app["speed_limits"] = load_speed_limits(args.config)
    app.router.add_get("/", index)
    app.router.add_get("/config", config_handler)
    app.router.add_get("/ws", websocket_handler)

    print(f"Serving http://0.0.0.0:{args.http_port}/ -- open this from a browser with the Xbox controller.")
    print(f"Relaying to teleop's ZeroMQ link on {args.zmq_host}:{COMMAND_PORT}/{TELEMETRY_PORT}.")
    web.run_app(app, host="0.0.0.0", port=args.http_port)


if __name__ == "__main__":
    main()
