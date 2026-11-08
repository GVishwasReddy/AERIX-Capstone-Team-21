# `drone_stack`

The main ROS-free Python package that runs on the Raspberry Pi companion computer. It joins configuration, a supervised node runtime, a message bus, flight-controller and sensor interfaces, navigation, delivery logic, the web GCS, and the optional MedBox bridge.

## Read next

- [Software architecture](../docs/guide/03-software-architecture.md)
- [Delivery mission](../docs/guide/04-delivery-mission.md)
- [Perception and avoidance](../docs/guide/05-perception-and-avoidance.md)
- [BLE handshake and security](../docs/guide/06-ble-handshake-security.md)
- [GCS](../docs/guide/07-ground-control-station.md)
- [Safety](../docs/guide/08-safety.md)

## Package map

| Package | Role |
|---|---|
| `bus/`, `msg/`, `srv/` | Internal pub/sub topics, typed messages, and service calls |
| `interfaces/` | MAVLink, LiDAR, rangefinder, and Firebase adapters |
| `nodes/` | Telemetry, fusion, navigation, delivery, avoidance, and diagnostics |
| `launch/` | Build the simulation or real-hardware node graph |
| `gcs/` | FastAPI server, command hub, camera pipeline, recording, and dashboard |
| `ble_handshake/` | Recipient verification and signed delivery receipts |
| `payload/` | Optional MedBox BLE bridge and wire protocol |
| `sim/` | Mock interfaces and simulated world |
| `novelty/` | Experimental/design-stage modules; see evidence notes before use |

Begin with the [repository guide](../docs/guide/README.md). Configure behavior in `config/`; never place credentials or live order data in source control.
