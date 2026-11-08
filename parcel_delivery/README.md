# Legacy parcel delivery prototype

This folder contains an earlier standalone delivery prototype. It is separate from the current ROS-free flight stack in [`drone_stack/`](../drone_stack/README.md). Use it for historical context or its isolated tests; do not treat its wiring or configuration as the active aircraft deployment.

The prototype includes a small Pi-side mission state machine, request validation, a LiDAR bridge, and simulator helpers under `pi/`, `scripts/`, and `tests/`.

For the current project flow, start with the [system guide](../docs/guide/README.md), [architecture](../docs/guide/03-software-architecture.md), and [safety guide](../docs/guide/08-safety.md).
