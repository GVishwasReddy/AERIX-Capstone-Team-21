# Configuration

Configuration is YAML based. `default.yaml` supplies shared values; `sim.yaml` and `real.yaml` select environment-specific behavior. Review the [configuration guide](../docs/CONFIGURATION.md) and [hardware guide](../docs/guide/02-hardware.md) before changing a value.

- `sim.yaml` is for software simulation. It does not prove hardware behavior.
- `real.yaml` contains hardware-specific ports and safety settings. Its presence does not mean the aircraft is cleared to fly.
- `novelty/` contains experimental module settings. Treat them according to the evidence tier.
- `*.example` files are templates. Keep real credentials in local, access-controlled files that are ignored by Git.
