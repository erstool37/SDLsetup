#!/usr/bin/env python3
"""The circulator transport, exercised entirely against a fake.

READ THIS BEFORE EDITING. The circulator is a custom MCU board behind an FTDI
FT232R USB-serial bridge, and on that bridge **DTR is capacitively coupled to
/RESET**. The OS asserts DTR when the port is opened, before any byte is sent,
so OPENING THE PORT HARDWARE-RESETS THE MICROCONTROLLER. There is no read-only
identify query and no way to "just check".

Consequently this file never opens a serial port, and no test in it may. Every
transport here is :class:`tools.circulator.link.FakeSerial`, which counts
``open_count`` precisely because each open is a reset event.

    python dev/tests/test_circulator_link.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tools.circulator.circulator import Circulator, CirculatorSettings  # noqa: E402
from tools.circulator.codec import SETPOINT_ADDRESS  # noqa: E402
from tools.circulator.link import FakeSerial, SerialLink, WriteResult  # noqa: E402
from tools.circulator.safety import (  # noqa: E402
    ActuationNotAllowed,
    CirculatorError,
    SafetyError,
)

# The three outcomes a write can have. `ok` is DERIVED and is True only for
# "confirmed", so a dry-run plan reports ok=False -- the honest answer to "was
# the setpoint written?". A caller who checks only `ok` cannot mistake a plan
# for a completed write, which is the failure this tri-state removes.
PLANNED, CONFIRMED, FAILED = "planned", "confirmed", "failed"

fails = 0
PORT = "/dev/ttyUSB-circulator-does-not-exist"


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        fails += 1


def blocked(fn, message: str, exc: type = CirculatorError) -> None:
    global fails
    try:
        value = fn()
    except exc as caught:
        print("[test] PASS: %s  -> %s" % (message, str(caught)[:70]))
        return
    except Exception as other:
        print("[test] FAIL: %s -- raised %s, not %s"
              % (message, type(other).__name__, exc.__name__))
        fails += 1
        return
    print("[test] FAIL: %s -- nothing raised, returned %r" % (message, value))
    fails += 1


def live(**kw) -> CirculatorSettings:
    base = {"port": PORT, "allow_actuation": True, "boot_settle_s": 0.0}
    base.update(kw)
    return CirculatorSettings(**base)


print("--- DRY RUN NEVER OPENS THE PORT (i.e. never resets the MCU) ---")
fake = FakeSerial()
dev = Circulator(CirculatorSettings(port=PORT, allow_actuation=False), transport=fake)
plan = dev.write_setpoint(dev.bound(25.0))
ok(plan.outcome == PLANNED, "a dry-run write reports outcome=%r" % PLANNED, plan.outcome)
ok(plan.ok is False,
   "and ok is False, because NOTHING WAS WRITTEN -- not a bug, the honest answer")
ok(plan.dry_run is True and plan.sent is False,
   "dry_run/sent are derived from outcome, so they cannot disagree with it")
ok(plan.address == SETPOINT_ADDRESS and plan.values == [0, 0, 0, 0x3940],
   "with the real address and the real registers", str(plan.values))
ok(plan.echo_address is None and plan.echo_count is None,
   "and no echo, because nothing was sent")
ok(fake.open_count == 0, "FakeSerial.open_count == 0 -- NO RESET EVENT")
ok(fake.frames == [], "and no frame reached the transport")

print("\n--- and never builds a real transport ---")
lazy = Circulator(CirculatorSettings(port=PORT, allow_actuation=False))
lazy.write_setpoint(lazy.bound(11.4))
ok(lazy.link._transport is None,
   "with allow_actuation off, no transport object is ever constructed")
ok("pymodbus" not in sys.modules and "serial" not in sys.modules,
   "and no vendor module was imported by doing it")

print("\n--- opening is an ACTUATING operation and is gated like one ---")
off = SerialLink(CirculatorSettings(port=PORT, allow_actuation=False), transport=FakeSerial())
blocked(off.open, "open() is refused while allow_actuation is off", ActuationNotAllowed)
ok(off._transport.open_count == 0, "nothing was opened by the refused call")

no_port = SerialLink(CirculatorSettings(port=None, allow_actuation=True),
                     transport=FakeSerial())
blocked(no_port.open, "open() is refused when no port is configured")
ok(no_port._transport.open_count == 0, "and again nothing was opened")

print("\n--- one open per session, no matter how often it is asked for ---")
fake = FakeSerial()
link = SerialLink(live(), transport=fake)
link.open()
link.open()
link.open()
ok(fake.open_count == 1, "three open() calls, ONE reset event", "open_count=%d" % fake.open_count)
ok(link.is_open, "and the link reports itself open")
link.close()
ok(fake.close_count == 1 and not link.is_open, "close() closes once")

print("\n--- a session records what it did ---")
fake = FakeSerial()
with SerialLink(live(), transport=fake) as link:
    result = link.write_registers(SerialLink.encode_frame(25.0))
ok(isinstance(result, WriteResult) and result.outcome == CONFIRMED,
   "a matching echo is outcome=%r" % CONFIRMED, result.outcome)
ok(result.ok is True, "and ok is True -- the ONLY state for which it is")
ok(result.dry_run is False and result.sent is True, "it was really sent")
ok(result.echo_address == SETPOINT_ADDRESS and result.echo_count == 4,
   "and the echo is reported, not just believed")
ok(fake.open_count == 1, "the context manager opened exactly once")
ok(fake.close_count == 1, "and closed on the way out")
ok(len(fake.frames) == 1 and fake.frames[0]["values"] == [0, 0, 0, 0x3940],
   "the frame the fake saw is the frame we asked for")
ok(fake.frames[0]["device_id"] == 1,
   "addressed to unit 1 -- INFERRED (library default; never specified in prior code)")

print("\n--- a write before the MCU has finished booting is refused ---")
fake = FakeSerial()
link = SerialLink(live(boot_settle_s=30.0), transport=fake)
link.open(settle=False)
ok(link.settle_remaining_s > 0, "the settle window is still open",
   "%.1f s left" % link.settle_remaining_s)
blocked(lambda: link.write_registers(SerialLink.encode_frame(20.0)),
        "a frame during the boot window raises", SafetyError)
ok(fake.frames == [], "and no frame was sent")
link.close()

print("\n--- writing without opening is refused rather than opening implicitly ---")
fake = FakeSerial()
link = SerialLink(live(), transport=fake)
blocked(lambda: link.write_registers(SerialLink.encode_frame(20.0)),
        "write_registers on a closed link raises")
ok(fake.open_count == 0, "and does NOT reset the MCU to satisfy the caller")

print("\n--- THE DEFECT: the prior code never checked what came back ---")
fake = FakeSerial(echo_address=999)
with SerialLink(live(), transport=fake) as link:
    bad = link.write_registers(SerialLink.encode_frame(25.0))
ok(bad.outcome == FAILED and bad.ok is False,
   "an echoed address of 999 is outcome=%r, ok=False" % FAILED, bad.outcome)
ok(bad.error and "999" in bad.error, "and says what came back", str(bad.error)[:60])
ok(bad.echo_address == 999, "the observed echo is preserved, not overwritten")

fake = FakeSerial(echo_count=2)
with SerialLink(live(), transport=fake) as link:
    bad = link.write_registers(SerialLink.encode_frame(25.0))
ok(bad.outcome == FAILED and bad.ok is False,
   "an echoed count of 2 for a 4-register write is outcome=%r" % FAILED)

fake = FakeSerial(error_response=True)
with SerialLink(live(), transport=fake) as link:
    bad = link.write_registers(SerialLink.encode_frame(25.0))
ok(bad.outcome == FAILED and bad.ok is False and bad.error,
   "a Modbus exception response is outcome=%r" % FAILED)

fake = FakeSerial(fail_write=True)
with SerialLink(live(), transport=fake) as link:
    bad = link.write_registers(SerialLink.encode_frame(25.0))
ok(bad.outcome == FAILED and bad.ok is False and bad.error,
   "a raised transport error is outcome=%r, not an exception" % FAILED)
ok(bad.dry_run is False and bad.sent is True, "and is not mistaken for a dry run")

print("\n--- the three states are exhaustive and a fourth cannot be built ---")
seen = set()
for outcome in (PLANNED, CONFIRMED, FAILED):
    seen.add(WriteResult(outcome=outcome, address=980, values=[0, 0, 0, 0]).outcome)
ok(seen == {PLANNED, CONFIRMED, FAILED}, "all three construct", ",".join(sorted(seen)))
ok(WriteResult.OUTCOMES == (PLANNED, CONFIRMED, FAILED), "and are the declared set")
blocked(lambda: WriteResult(outcome="ok", address=980, values=[0]),
        "a typo'd outcome raises rather than reading as a silent ok=False")
blocked(lambda: WriteResult(outcome=CONFIRMED, address=980, values=[0], error="boom"),
        "a confirmed write cannot also carry an error")
plan_dict = WriteResult(outcome=PLANNED, address=980, values=[0]).as_dict()
ok(plan_dict["ok"] is False and plan_dict["outcome"] == PLANNED,
   "as_dict carries the derived ok and dry_run, so a status page cannot lose them")
ok(plan_dict["dry_run"] is True and plan_dict["sent"] is False, "all four agree")

print("\n--- the dashboard cannot open the port, not even transitively ---")
from tools.circulator.node import CirculatorNode  # noqa: E402

node = CirculatorNode(settings=live())
ok("open" not in node.commands() and "close" not in node.commands(),
   "open/close are absent from the dashboard surface", ",".join(node.commands()))
refusal = node.command("set_setpoint", target_c=25.0)
ok(refusal["ok"] is False and "refused" in refusal,
   "set_setpoint on a closed link is REFUSED, not an error", str(refusal)[:64])
ok("RESET" in refusal.get("refused", ""), "and the refusal names the MCU reset")
ok(node.command("open")["ok"] is False, "asking for open anyway is not honoured")
dry_node = CirculatorNode(settings=CirculatorSettings(port=PORT))
planned = dry_node.command("set_setpoint", target_c=25.0)
ok(planned["ok"] is True and planned["outcome"] == PLANNED,
   "a dry-run node command RAN (envelope ok) but only PLANNED", str(planned["outcome"]))
ok(planned["result"]["ok"] is False,
   "while the write itself still reports ok=False -- nothing was written")
ok(dry_node.status()["open_count"] == 0,
   "and open_count stays on the status card, at zero")

print("\n--- a failed write REPORTS; it does not decide ---")
ok(isinstance(bad, WriteResult),
   "the caller gets data back and chooses what to do -- tools report, scripts decide")

print("\n--- port_present() uses stat only. It NEVER opens. ---")
exploding = FakeSerial(fail_open=True)
link = SerialLink(live(), transport=exploding)
presence = link.port_present()
ok(presence["present"] is False, "a port that is not there reports absent", str(presence))
ok(exploding.open_count == 0, "port_present did not call connect()")
ok(presence["port"] == PORT, "and reports which path it looked at")
here = SerialLink(live(port="/dev/null"), transport=FakeSerial(fail_open=True))
ok(here.port_present()["present"] is True, "/dev/null exists, and checking it opened nothing")
ok(SerialLink(CirculatorSettings(port=None), transport=FakeSerial()).port_present()["present"]
   is False, "no configured port reports absent without raising")

print("\n--- a failing open is a failure, not a silent half-open link ---")
link = SerialLink(live(), transport=FakeSerial(fail_open=True))
blocked(link.open, "a transport that cannot connect raises")
ok(not link.is_open, "and the link does not claim to be open")

print("\n--- F2c: the raw wire takes ONLY a codec-built frame, not (address, values) ---")
# The prior write_registers(address, values) accepted any list at any address:
# a 1-register partial write, a PLC-codec pair, NaN bits. Now it takes only a
# private _SetpointFrame built by encode_frame at the canonical address 980.
frame = SerialLink.encode_frame(25.0)
ok(frame.address == SETPOINT_ADDRESS and frame.values == (0, 0, 0, 0x3940),
   "encode_frame builds the canonical 4-register frame at address 980 (H3: tuple)",
   "%d %r" % (frame.address, frame.values))
with SerialLink(live(), transport=FakeSerial()) as link:
    blocked(lambda: link.write_registers(SETPOINT_ADDRESS, [0, 0, 0, 0x3940]),
            "a raw (address, values) call no longer even fits the signature",
            TypeError)
    blocked(lambda: link.write_registers([0, 0, 0, 0x3940]),
            "a bare register list is refused -- not a codec-built frame", SafetyError)
    blocked(lambda: link.write_registers([0x3940]),
            "a 1-register partial write cannot even be expressed", SafetyError)
    good = link.write_registers(SerialLink.encode_frame(25.0))
    ok(good.outcome == CONFIRMED, "the codec-built frame is accepted and confirmed")
# NaN cannot be encoded into a frame at all: the codec refuses it upstream.
blocked(lambda: SerialLink.encode_frame(float("nan")),
        "encode_frame refuses NaN, so NaN bits cannot reach the wire", ValueError)
# And the public API surface no longer re-exports the raw transport.
import tools.circulator.api as _circ_api  # noqa: E402

ok("SerialLink" not in _circ_api.__all__ and not hasattr(_circ_api, "SerialLink"),
   "SerialLink is NOT re-exported from tools.circulator.api")

print("\n--- F11: a matching echo with a non-FC16 function code is NOT confirmed ---")
# This firmware returns MALFORMED exception frames -- function code 0x81 where an
# FC3 exception must be 0x83. A frame whose address and count line up but whose
# function code is not 0x10 must be judged FAILED, never confirmed.
fake = FakeSerial(function_code=0x81)   # matching address/count, isError False
with SerialLink(live(), transport=fake) as link:
    bad = link.write_registers(SerialLink.encode_frame(25.0))
ok(bad.outcome == FAILED and bad.ok is False,
   "a malformed 0x81 frame with matching address/count is FAILED", bad.outcome)
ok(bad.error and "0x81" in bad.error and "0x10" in bad.error,
   "and the error names the bad function code and the expected FC16", str(bad.error)[:90])
fake = FakeSerial(function_code=0x10)
with SerialLink(live(), transport=fake) as link:
    good = link.write_registers(SerialLink.encode_frame(25.0))
ok(good.outcome == CONFIRMED, "the real FC16 (0x10) response still confirms", good.outcome)

print("\n--- F10: a failed open still COUNTS the MCU reset it caused ---")
# connect() asserts DTR (resetting the MCU) before it can report failure, so a
# failed open must record reset_attempted, distinct from a connection being
# established -- otherwise diagnostics claim no reset happened and a retry
# resets the board again unaccounted.
link = SerialLink(live(), transport=FakeSerial(fail_open=True))
blocked(link.open, "a connect that raises is a failure")
ok(link.reset_attempts == 1, "but the reset it caused is COUNTED", "reset_attempts=%d" % link.reset_attempts)
ok(link.open_count == 0, "while open_count stays 0 -- no connection was established")
ok(not link.is_open, "and the link is not left half-open")
pp = link.port_present()
ok(pp.get("reset_attempts") == 1 and pp.get("open_count") == 0,
   "port_present surfaces both counters", str({k: pp.get(k) for k in ("reset_attempts", "open_count")}))
# A retry resets it again, and that is visible as a second attempt.
blocked(link.open, "a retry after a failed open is refused too")
ok(link.reset_attempts == 2, "and is counted as a SECOND reset attempt", "reset_attempts=%d" % link.reset_attempts)

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
