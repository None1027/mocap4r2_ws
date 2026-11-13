#!/usr/bin/env python3
"""Robokit API 2010 over TCP. Python 3.10+, standard library only."""

import argparse
import json
import math
import socket
import struct
import sys
import time

if __package__:
    from .limits import DEFAULT_HOST, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, limit_velocity, validate_limits
else:
    from limits import DEFAULT_HOST, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, limit_velocity, validate_limits


HEADER = struct.Struct("!BBHIH6s")
MOTION_REQUEST = 2010
MOTION_RESPONSE = 12010
MAX_BODY_BYTES = 1024 * 1024


class ProtocolError(RuntimeError):
    pass


class RobokitClient:
    """One persistent connection, with one request in flight. Single-thread use."""

    def __init__(self, host=DEFAULT_HOST, port=19205, version=1, timeout=0.2,
                 max_linear_speed=MAX_LINEAR_SPEED, max_angular_speed=MAX_ANGULAR_SPEED):
        if not host or not 1 <= port <= 65535:
            raise ValueError("A robot IP/hostname and a valid port are required")
        if version not in (1, 2):
            raise ValueError("version must be 1 (RBK3.4) or 2 (RBK3.5)")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        validate_limits(max_linear_speed, max_angular_speed)
        self.host = host
        self.port = port
        self.version = version
        self.timeout = timeout
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.sequence = 0
        self.has_sent = False
        self.sock = None

    def connect(self):
        if self.sock is None:
            connection = socket.create_connection(
                (self.host, self.port), timeout=self.timeout
            )
            try:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                connection.close()
                raise
            self.sock = connection

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _recv_exact(self, length, deadline):
        result = bytearray()
        while len(result) < length:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Robokit response deadline exceeded")
            self.sock.settimeout(remaining)
            chunk = self.sock.recv(length - len(result))
            if not chunk:
                raise ConnectionError("Robot closed the TCP connection")
            result.extend(chunk)
        return bytes(result)

    def motion(self, vx=0.0, vy=0.0, w=0.0, duration_ms=300):
        """Robot-frame m/s, m/s, rad/s. Positive w is counterclockwise."""
        vx, vy, w = limit_velocity(
            vx, vy, w, self.max_linear_speed, self.max_angular_speed
        )
        if type(duration_ms) is not int or not 1 <= duration_ms <= 5000:
            raise ValueError("duration_ms must be an integer in 1..5000")
        body = json.dumps(
            {"vx": vx, "vy": vy, "w": w, "duration": duration_ms},
            separators=(",", ":"), allow_nan=False,
        ).encode("utf-8")
        self.connect()
        self.sequence = (self.sequence + 1) & 0xFFFF
        packet = HEADER.pack(
            0x5A, self.version, self.sequence, len(body), MOTION_REQUEST, bytes(6)
        ) + body
        try:
            deadline = time.monotonic() + self.timeout
            self.sock.settimeout(self.timeout)
            self.has_sent = True
            self.sock.sendall(packet)
            header = self._recv_exact(HEADER.size, deadline)
            sync, version, sequence, length, api_type, reserved = HEADER.unpack(header)
            if (sync, version, sequence, api_type) != (
                0x5A, self.version, self.sequence, MOTION_RESPONSE
            ):
                raise ProtocolError("Unexpected response header: " + header.hex(" "))
            if length > MAX_BODY_BYTES:
                raise ProtocolError("Response body exceeds 1 MiB")
            if reserved[2:4] != bytes(2) or reserved[4] != 0:
                raise ProtocolError("Unexpected routed or compressed response")
            payload = self._recv_exact(length, deadline)
            response = json.loads(payload.decode("utf-8")) if payload else {}
            if not isinstance(response, dict):
                raise ProtocolError("Response JSON must be an object")
            # The supplied documentation allows ret_code and even the body to be absent.
            if response.get("ret_code", 0) != 0:
                raise ProtocolError("Robot rejected API 2010: " + repr(response))
            return response
        except BaseException:
            # A partial response makes stream reuse ambiguous.
            self.close()
            raise

    def stop(self, duration_ms=300):
        return self.motion(0.0, 0.0, 0.0, duration_ms)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", "--hostname", default=DEFAULT_HOST,
                        help=f"Robot IP or hostname (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=19205)
    parser.add_argument("--version", type=int, choices=(1, 2), default=1)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--w", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="0 sends once; positive values refresh for this long")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--duration-ms", type=int, default=300)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--max-linear-speed", type=float, default=MAX_LINEAR_SPEED,
                        help="Planar vector speed limit in m/s, up to 0.5")
    parser.add_argument("--max-angular-speed", type=float, default=MAX_ANGULAR_SPEED,
                        help=f"Yaw rate limit in rad/s, up to {MAX_ANGULAR_SPEED} (5 deg/s)")
    args = parser.parse_args()
    numeric = (args.vx, args.vy, args.w, args.seconds, args.rate, args.timeout)
    if not all(math.isfinite(value) for value in numeric):
        parser.error("All numeric arguments must be finite")
    if args.seconds < 0 or args.rate <= 0 or args.timeout <= 0:
        parser.error("seconds must be nonnegative; rate and timeout must be positive")
    if not 1 <= args.duration_ms <= 5000:
        parser.error("duration-ms must be in 1..5000")
    if args.seconds > 0 and 1000.0 / args.rate >= args.duration_ms:
        parser.error("The refresh period must be shorter than duration-ms")
    try:
        velocity = limit_velocity(
            args.vx, args.vy, args.w, args.max_linear_speed, args.max_angular_speed
        )
        client = RobokitClient(
            args.host, args.port, args.version, args.timeout,
            args.max_linear_speed, args.max_angular_speed,
        )
    except ValueError as error:
        parser.error(str(error))
    if velocity != (args.vx, args.vy, args.w):
        print(f"Velocity clamped: vx={velocity[0]:.6g}, vy={velocity[1]:.6g} m/s, "
              f"w={velocity[2]:.6g} rad/s")
    exit_code = 0
    try:
        start = time.monotonic()
        response = client.motion(*velocity, duration_ms=args.duration_ms)
        print("API 2010 accepted:", json.dumps(response, ensure_ascii=False))
        if args.seconds > 0:
            end = start + args.seconds
            while time.monotonic() < end:
                time.sleep(min(1.0 / args.rate, max(0.0, end - time.monotonic())))
                if time.monotonic() >= end:
                    break
                client.motion(*velocity, duration_ms=args.duration_ms)
        else:
            # Let the finite single command finish before sending an explicit stop.
            time.sleep(args.duration_ms / 1000.0)
    except KeyboardInterrupt:
        exit_code = 130
    except (OSError, RuntimeError, ValueError) as error:
        print("Motion failed:", error, file=sys.stderr)
        exit_code = 1
    finally:
        try:
            client.stop(args.duration_ms)
            print("Zero-velocity command accepted")
        except (OSError, RuntimeError, ValueError) as error:
            print("Stop delivery unconfirmed:", error, file=sys.stderr)
            exit_code = exit_code or 1
        client.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
