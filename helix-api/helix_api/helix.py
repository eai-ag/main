"""Main Helix class for robot arm control."""

import roslibpy
from enum import Enum
from typing import Optional


class ControlMode(Enum):
    """Control modes for the Helix robot arm."""
    NONE = "none"
    POSITION_CONTROL = "position_control"
    CURRENT_CONTROL = "current_control"
    VELOCITY_CONTROL = "velocity_control"


class Helix:
    """
    Helix robot arm controller.

    This class provides a simple interface to connect to and control
    a Helix robot arm via ROS using roslibpy.

    Example:
        >>> from helix_api import Helix
        >>> helix = Helix("eai-helix-0.local")
        >>> helix.connect()
        >>> helix.switch_control_mode("position_control")
        >>> helix.disconnect()
    """

    def __init__(self, host: str, port: int = 9090):
        """
        Initialize Helix robot arm connection.

        Args:
            host: The hostname or IP address of the robot (e.g., "eai-helix-0.local")
            port: The ROSBridge websocket port (default: 9090)
        """
        self.host = host
        self.port = port
        self.client: Optional[roslibpy.Ros] = None
        self._control_mode_service: Optional[roslibpy.Service] = None
        self._current_mode: Optional[ControlMode] = None

    def connect(self, timeout: float = 5.0) -> bool:
        """
        Connect to the robot.

        Args:
            timeout: Connection timeout in seconds (default: 5.0)

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run(timeout=timeout)

            # Initialize control mode service
            self._control_mode_service = roslibpy.Service(
                self.client,
                '/helix/set_control_mode',
                'std_srvs/SetString'
            )

            return self.is_connected()
        except Exception as e:
            print(f"Failed to connect to {self.host}:{self.port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from the robot."""
        if self.client and self.client.is_connected:
            self.client.terminate()
            self.client = None
            self._control_mode_service = None

    def is_connected(self) -> bool:
        """
        Check if connected to the robot.

        Returns:
            True if connected, False otherwise
        """
        return self.client is not None and self.client.is_connected

    def switch_control_mode(self, mode: str) -> bool:
        """
        Switch the robot control mode.

        Args:
            mode: The control mode. Valid values:
                  - "none": No control
                  - "position_control": Position control mode
                  - "current_control": Current/torque control mode
                  - "velocity_control": Velocity control mode

        Returns:
            True if mode switch successful, False otherwise

        Raises:
            ValueError: If mode is invalid
            ConnectionError: If not connected to robot
        """
        if not self.is_connected():
            raise ConnectionError("Not connected to robot. Call connect() first.")

        # Validate mode
        try:
            control_mode = ControlMode(mode)
        except ValueError:
            valid_modes = [m.value for m in ControlMode]
            raise ValueError(
                f"Invalid control mode '{mode}'. Valid modes: {valid_modes}"
            )

        try:
            request = roslibpy.ServiceRequest({'data': mode})
            response = self._control_mode_service.call(request, timeout=5.0)

            if response.get('success', False):
                self._current_mode = control_mode
                return True
            else:
                print(f"Failed to switch mode: {response.get('message', 'Unknown error')}")
                return False

        except Exception as e:
            print(f"Error switching control mode: {e}")
            return False

    def get_control_mode(self) -> Optional[str]:
        """
        Get the current control mode.

        Returns:
            The current control mode as a string, or None if not set
        """
        return self._current_mode.value if self._current_mode else None

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

    def __repr__(self) -> str:
        """String representation."""
        status = "connected" if self.is_connected() else "disconnected"
        return f"Helix(host='{self.host}', port={self.port}, status='{status}')"
