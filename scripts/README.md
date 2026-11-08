# Operator and development scripts

This directory contains helpers for simulation, setup, diagnostics, evidence capture, and hardware-specific maintenance. Read a script before running it; many tools can communicate with the flight controller or alter parameters.

| Script group | Examples | Use |
|---|---|---|
| Start stack | `run_sim.sh`, `run_real.sh`, `start_gcs.sh` | Launch the selected profile |
| Preflight | `preflight_check.py` | Report configured and available checks |
| Simulation | `sim_phone_pick.py`, `inject_order.py` | Exercise software flows without flight |
| Hardware diagnostics | `read_*`, `aux_pin_check.py`, `lidar_orientation_check.py` | Inspect attached hardware or settings |
| Deployment | `deploy_to_pi.sh` | Transfer selected source files to the Pi |
| Evidence | `trial_log.py`, `verify_protocol.sh` | Record or check development evidence |

See [setup, deployment, and testing](../docs/guide/09-setup-deployment-testing.md). Simulation scripts are not flight tests.
