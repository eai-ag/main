import roslibpy
from enum import Enum
from typing import Optional


class ControlMode(Enum):
    NONE = "none"
    POSITION_CONTROL = "position_control"
    CURRENT_CONTROL = "current_control"
    VELOCITY_CONTROL = "velocity_control"


class Helix:
    def __init__(self, host: str, port: int = 9090):
        self.host = host
        self.port = port
        self.client: Optional[roslibpy.Ros] = None
        self._control_mode_service: Optional[roslibpy.Service] = None
        self._current_mode: Optional[ControlMode] = None

    def connect(self, timeout: float = 5.0) -> bool:
        try:
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run(timeout=timeout)

            self._control_mode_service = roslibpy.Service(self.client,'/helix/dynamixel_driver_node/switch_control_mode','std_srvs/SetString')

            return self.is_connected()
        except Exception as e:
            print(f"Failed to connect to {self.host}:{self.port}: {e}")
            return False

    def disconnect(self):
        if self.client and self.client.is_connected:
            self.client.terminate()
            self.client = None
            self._control_mode_service = None

    def is_connected(self) -> bool:
        return self.client is not None and self.client.is_connected

    def switch_control_mode(self, mode: str) -> bool:
        if not self.is_connected():
            raise ConnectionError("Not connected to robot. Call connect() first.")

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
        return self._current_mode.value if self._current_mode else None

    def __repr__(self) -> str:
        status = "connected" if self.is_connected() else "disconnected"
        return f"Helix(host='{self.host}', port={self.port}, status='{status}')"
