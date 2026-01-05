# Helix API

A minimal Python package to control Helix robot arm via ROS using roslibpy.

## Installation

### Using pip

```bash
pip install -e .
```

### Using uv (recommended for development)

```bash
# Install the package in development mode
uv pip install -e .

# Or install with dev dependencies
uv pip install -e ".[dev]"
```

## Requirements

- Python >= 3.7
- roslibpy >= 1.0.0
- A running ROSBridge server on the Helix robot (default port: 9090)

## Quick Start

```python
from helix_api import Helix

# Create a Helix instance
helix = Helix("eai-helix-0.local")

# Connect to the robot
helix.connect()

# Switch control mode
helix.switch_control_mode("position_control")

# Get current control mode
print(helix.get_control_mode())

# Disconnect when done
helix.disconnect()
```

## Using Context Manager

```python
from helix_api import Helix

# Automatically connects and disconnects
with Helix("eai-helix-0.local") as helix:
    helix.switch_control_mode("velocity_control")
    # Do work...
```

## Control Modes

The following control modes are supported:

- `"none"`: No control
- `"position_control"`: Position control mode
- `"current_control"`: Current/torque control mode
- `"velocity_control"`: Velocity control mode

## API Reference

### `Helix(host, port=9090)`

Create a new Helix robot controller.

**Parameters:**
- `host` (str): The hostname or IP address of the robot (e.g., "eai-helix-0.local")
- `port` (int, optional): The ROSBridge websocket port (default: 9090)

### Methods

#### `connect(timeout=5.0) -> bool`

Connect to the robot.

**Parameters:**
- `timeout` (float, optional): Connection timeout in seconds (default: 5.0)

**Returns:**
- `bool`: True if connection successful, False otherwise

#### `disconnect()`

Disconnect from the robot.

#### `is_connected() -> bool`

Check if connected to the robot.

**Returns:**
- `bool`: True if connected, False otherwise

#### `switch_control_mode(mode: str) -> bool`

Switch the robot control mode.

**Parameters:**
- `mode` (str): The control mode ("none", "position_control", "current_control", or "velocity_control")

**Returns:**
- `bool`: True if mode switch successful, False otherwise

**Raises:**
- `ValueError`: If mode is invalid
- `ConnectionError`: If not connected to robot

#### `get_control_mode() -> Optional[str]`

Get the current control mode.

**Returns:**
- `str | None`: The current control mode as a string, or None if not set

## Development

### Setup with uv

```bash
# Install uv if you haven't already
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install dependencies
uv pip install -r requirements.txt

# Or install in editable mode with dev dependencies
uv pip install -e ".[dev]"
```

### Running Tests

```bash
# Using pytest directly
pytest tests/

# Or with uv
uv run pytest tests/

# Run with verbose output
pytest -v tests/

# Run specific test file
pytest tests/test_helix.py
```

### Project Structure

```
helix-api/
├── helix_api/
│   ├── __init__.py
│   └── helix.py
├── tests/
│   ├── __init__.py
│   └── test_helix.py
├── setup.py
├── pyproject.toml
├── requirements.txt
└── README.md
```

## License

MIT
