# environment node (vacant)

Droplet-container humidity / temperature control. **Not implemented yet.**
"Further connected" — expected to interlock with experiments (e.g. hold a
setpoint during a run, or gate the arm/camera on a stable environment).

Planned: `read` (humidity, temperature), `set_humidity`, `set_temperature`;
status publishes current vs setpoint and in-range flag.

To implement: identify the controller (e.g. a PID humidity/temp box over
serial/Modbus/USB, or a microcontroller). Record interface + setpoint limits
here. Expose an `in_range` boolean other nodes can subscribe to on the bus.
