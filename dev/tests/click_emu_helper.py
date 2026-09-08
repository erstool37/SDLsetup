#!/usr/bin/env python3
"""A dependency-free CLICK PLC emulator: minimal Modbus TCP over a socket.

Copied from the validated standalone emulator and adjusted only enough to
import cleanly and to take its port on the command line.

**Why the framing is hand-written rather than using pymodbus's server.** The
pymodbus server API moves between versions -- 3.15 removed
``ModbusSlaveContext`` (now ``ModbusDeviceContext``) and its
``ModbusSequentialDataBlock`` rejects a start address of 0, which is exactly
what a DF block at 28672 needs. The *client* side of the test uses the real
pymodbus, so interoperability is still what is being verified; only the server
is ours.

The register bank is seeded with one real logged row, 2025-07-26 15:52:32::

    25.000 Temp SP   95.000 RH SP   24.866 Temp Filtered   95.180 RH Filtered
    24.774 Temp PID output   25.340 RH PID output

Run it as ``python click_emu_helper.py [port]`` (default 15020). It is not
named ``test_*`` on purpose: ``dev/test.sh`` globs ``test_*.py`` and must not
start a server as if it were a test.
"""
from __future__ import annotations

import socket
import socketserver
import struct
import sys
import threading

DF_BASE = 28672
C1_COIL = 16384
DEFAULT_PORT = 15020
ILLEGAL_ADDR = 0x02
ILLEGAL_FUNCTION = 0x01


def df_addr(n: int) -> int:
    return DF_BASE + 2 * (n - 1)


def f32_regs(v: float) -> tuple[int, int]:
    """CLICK order: bytes big-endian inside a word, LSW at the LOWER address."""
    hi, lo = struct.unpack(">HH", struct.pack(">f", v))
    return lo, hi


#: DF number -> value. The logged row, plus the gains and bounds around it.
VALUES = {
    1: 95.180, 2: 24.866, 3: 0.0, 4: 0.0, 5: 25.000, 6: 95.000,
    7: 0.134, 8: -0.180, 9: 24.774, 10: 25.340,
    11: 2.5, 12: 120.0, 13: 1.8, 14: 90.0, 15: 0.140, 16: -0.200, 17: 0.2,
    18: 24.866, 19: 95.180, 20: 30.0, 21: 0.0, 22: 100.0, 23: 0.0,
}

HR: dict[int, int] = {}
COILS: dict[int, int] = {C1_COIL: 1}
LOCK = threading.Lock()

for _n, _v in VALUES.items():
    _lo, _hi = f32_regs(_v)
    HR[df_addr(_n)] = _lo
    HR[df_addr(_n) + 1] = _hi


def pdu(fc: int, body: bytes) -> bytes:
    return struct.pack(">B", fc) + body


def err(fc: int, code: int) -> bytes:
    return struct.pack(">BB", fc | 0x80, code)


def handle(unit: int, data: bytes) -> bytes:
    fc = data[0]
    with LOCK:
        if fc == 3:                                    # read holding registers
            addr, cnt = struct.unpack(">HH", data[1:5])
            if not (1 <= cnt <= 125):
                return err(fc, ILLEGAL_ADDR)
            vals = [HR.get(addr + i, 0) for i in range(cnt)]
            body = struct.pack(">B", cnt * 2) + b"".join(
                struct.pack(">H", v) for v in vals)
            return pdu(3, body)
        if fc == 1:                                    # read coils
            addr, cnt = struct.unpack(">HH", data[1:5])
            nbytes = (cnt + 7) // 8
            bits = bytearray(nbytes)
            for i in range(cnt):
                if COILS.get(addr + i, 0):
                    bits[i // 8] |= 1 << (i % 8)
            return pdu(1, struct.pack(">B", nbytes) + bytes(bits))
        if fc == 6:                                    # write single register
            addr, val = struct.unpack(">HH", data[1:5])
            HR[addr] = val
            return pdu(6, struct.pack(">HH", addr, val))
        if fc == 16:                                   # write multiple registers
            addr, cnt, nb = struct.unpack(">HHB", data[1:6])
            vals = struct.unpack(">%dH" % cnt, data[6:6 + nb])
            for i, v in enumerate(vals):
                HR[addr + i] = v
            return pdu(16, struct.pack(">HH", addr, cnt))
        if fc == 5:                                    # write single coil
            addr, val = struct.unpack(">HH", data[1:5])
            COILS[addr] = 1 if val == 0xFF00 else 0
            return pdu(5, struct.pack(">HH", addr, val))
    return err(fc, ILLEGAL_FUNCTION)


class Handler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        buf = b""
        while True:
            try:
                chunk = self.request.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while len(buf) >= 6:
                tid, pid, ln = struct.unpack(">HHH", buf[:6])
                if len(buf) < 6 + ln:
                    break
                frame = buf[6:6 + ln]
                buf = buf[6 + ln:]
                unit, body = frame[0], frame[1:]
                resp = handle(unit, body)
                out = struct.pack(">HHH", tid, pid, len(resp) + 1) + bytes([unit]) + resp
                try:
                    self.request.sendall(out)
                except OSError:
                    return


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    port = int(args[0]) if args else DEFAULT_PORT
    print("[emu] CLICK emulator listening on 127.0.0.1:%d" % port, flush=True)
    Server(("127.0.0.1", port), Handler).serve_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
