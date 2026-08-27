#!/usr/bin/env python3
"""Joystick -> ChassisSpeeds bridge for otto_suite's ZeroMQ link (plan.md
Layer 4). Reads one joystick, publishes a ChassisCommandWire at a fixed
rate, and prints telemetry (odometry + fault flag) received back from the
C++ real-time core.

Wire format must match src/bridge/messages.hpp exactly (little-endian,
packed, no padding):
  command:   3x double (vx_mps, vy_mps, omega_rad_per_s)          = 24 bytes
  telemetry: 3x double (vx_mps, vy_mps, omega_rad_per_s) + 1 byte = 25 bytes

The axis-to-motion mapping below (left stick = translate, right stick X =
rotate) is a common controller convention, NOT verified against your
specific joystick's actual axis numbering -- run `jstest /dev/input/js0`
first if motion doesn't match the stick you moved, and adjust the
get_axis() indices.

Usage:
  pip install pygame pyzmq
  python3 teleop_joystick.py [--host HOST] [--max-speed-mps 0.15] [--max-omega-deg-s 30]
"""
import argparse
import math
import os
import struct
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import pygame  # noqa: E402
import zmq  # noqa: E402

COMMAND_PORT = 5555
TELEMETRY_PORT = 5556
SEND_HZ = 50
DEADZONE = 0.1


def apply_deadzone(value, deadzone=DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="localhost", help="Host running the C++ real-time core")
    parser.add_argument("--max-speed-mps", type=float, default=0.15)
    parser.add_argument("--max-omega-deg-s", type=float, default=30.0)
    parser.add_argument("--joystick-index", type=int, default=0)
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("error: no joystick found")
    joystick = pygame.joystick.Joystick(args.joystick_index)
    joystick.init()
    print(f"Using joystick: {joystick.get_name()}")

    context = zmq.Context()
    command_pub = context.socket(zmq.PUB)
    command_pub.connect(f"tcp://{args.host}:{COMMAND_PORT}")
    telemetry_sub = context.socket(zmq.SUB)
    telemetry_sub.connect(f"tcp://{args.host}:{TELEMETRY_PORT}")
    telemetry_sub.setsockopt(zmq.SUBSCRIBE, b"")
    telemetry_sub.setsockopt(zmq.CONFLATE, 1)

    max_omega_rad_s = math.radians(args.max_omega_deg_s)
    period_s = 1.0 / SEND_HZ
    print(
        f"Publishing commands to tcp://{args.host}:{COMMAND_PORT}, "
        f"max_speed={args.max_speed_mps} m/s, max_omega={args.max_omega_deg_s} deg/s. Ctrl+C to stop."
    )

    last_print = 0.0
    try:
        while True:
            loop_start = time.monotonic()
            pygame.event.pump()

            vy = -apply_deadzone(joystick.get_axis(0)) * args.max_speed_mps  # left stick X -> strafe
            vx = -apply_deadzone(joystick.get_axis(1)) * args.max_speed_mps  # left stick Y -> forward
            omega = -apply_deadzone(joystick.get_axis(3)) * max_omega_rad_s  # right stick X -> rotate

            command_pub.send(struct.pack("<ddd", vx, vy, omega))

            try:
                data = telemetry_sub.recv(flags=zmq.NOBLOCK)
                odom_vx, odom_vy, odom_omega, any_fault = struct.unpack("<dddB", data)
                if loop_start - last_print > 0.5:
                    print(
                        f"cmd(vx={vx:+.3f} vy={vy:+.3f} w={omega:+.2f})  "
                        f"odom(vx={odom_vx:+.3f} vy={odom_vy:+.3f} w={odom_omega:+.2f})  "
                        f"fault={'YES' if any_fault else 'no'}"
                    )
                    last_print = loop_start
            except zmq.Again:
                pass

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, period_s - elapsed))
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        command_pub.close()
        telemetry_sub.close()
        context.term()


if __name__ == "__main__":
    main()
