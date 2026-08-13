# opentrons node (vacant)

Opentrons liquid-handling robot. **Not implemented yet** (the Opentrons Python
SDK exists — wire it here).

Planned commands: `run_protocol`, `home`, `pause`, `resume`, `estop`; status
should surface run state, current step, and pipette/labware info.

To implement: use the `opentrons` Python package (or the OT-2/Flex HTTP API at
`http://<robot-ip>:31950`). Prefer the HTTP API for run control so this node
stays decoupled. Record robot IP/serial + API token here once known. Treat all
motion as live hardware — gate behind explicit enable, like the arm node.
