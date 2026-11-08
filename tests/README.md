# Tests

The test suite covers the bus, configuration, navigation, delivery, sensor interfaces, the GCS, BLE verification, and the optional MedBox protocol. Most tests use mocks or simulation and do not exercise aircraft hardware.

From a Python 3.11 environment with development requirements installed:

```bash
python -m pytest -q
```

For a focused test, pass a specific file, such as `tests/test_medbox.py`. A passing test suite is not evidence that the aircraft is safe to fly. Read the [evidence guide](../evidence/EVIDENCE_TIERS.md) and [safety guide](../docs/guide/08-safety.md).
