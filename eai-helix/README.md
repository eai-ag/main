# Embodied AI Helix API

Python package to control Embodied AI's Helix robot arm.

**Package name:** `embodiedai-helix-api`
**Import name:** `embodiedai_helix_api`

## Installation

```bash
uv pip install -e .
```

## Usage

```python
from embodiedai_helix_api import Helix

helix = Helix("eai-helix-0.local")
helix.connect()
helix.switch_control_mode("position_control")
helix.disconnect()
```

## Control Modes

- `"none"`
- `"position_control"`
- `"current_control"`
- `"velocity_control"`

## Development

```bash
# Install with dev dependencies
uv pip install -e ".[dev]"

# Run tests
pytest tests/
```

## Requirements

- Python >= 3.7
- roslibpy >= 1.0.0
- ROSBridge server running on robot (port 9090)
