import pytest
from unittest.mock import Mock, patch, MagicMock
from embodiedai_helix_api import Helix, ControlMode


class TestHelixInitialization:
    """Test Helix initialization."""

    def test_init_with_default_port(self):
        """Test initialization with default port."""
        helix = Helix("eai-helix-0.local")
        assert helix.host == "eai-helix-0.local"
        assert helix.port == 9090
        assert helix.client is None

    def test_init_with_custom_port(self):
        """Test initialization with custom port."""
        helix = Helix("eai-helix-0.local", port=9091)
        assert helix.host == "eai-helix-0.local"
        assert helix.port == 9091

    def test_repr(self):
        """Test string representation."""
        helix = Helix("eai-helix-0.local")
        repr_str = repr(helix)
        assert "eai-helix-0.local" in repr_str
        assert "9090" in repr_str
        assert "disconnected" in repr_str


class TestHelixConnection:
    """Test Helix connection methods."""

    @patch('embodiedai_helix_api.helix.roslibpy.Ros')
    @patch('embodiedai_helix_api.helix.roslibpy.Service')
    def test_connect_success(self, mock_service, mock_ros):
        """Test successful connection."""
        # Setup mocks
        mock_client = Mock()
        mock_client.is_connected = True
        mock_ros.return_value = mock_client

        helix = Helix("eai-helix-0.local")
        result = helix.connect()

        assert result is True
        assert helix.is_connected() is True
        mock_ros.assert_called_once_with(host="eai-helix-0.local", port=9090)
        mock_client.run.assert_called_once()

    @patch('embodiedai_helix_api.helix.roslibpy.Ros')
    def test_connect_failure(self, mock_ros):
        """Test connection failure."""
        mock_ros.side_effect = Exception("Connection failed")

        helix = Helix("eai-helix-0.local")
        result = helix.connect()

        assert result is False
        assert helix.is_connected() is False

    @patch('embodiedai_helix_api.helix.roslibpy.Ros')
    @patch('embodiedai_helix_api.helix.roslibpy.Service')
    def test_disconnect(self, mock_service, mock_ros):
        """Test disconnection."""
        mock_client = Mock()
        mock_client.is_connected = True
        mock_ros.return_value = mock_client

        helix = Helix("eai-helix-0.local")
        helix.connect()
        helix.disconnect()

        mock_client.terminate.assert_called_once()
        assert helix.client is None


class TestControlMode:
    """Test control mode switching."""

    @patch('embodiedai_helix_api.helix.roslibpy.Ros')
    @patch('embodiedai_helix_api.helix.roslibpy.Service')
    def test_switch_control_mode_success(self, mock_service_class, mock_ros):
        """Test successful control mode switch."""
        # Setup mocks
        mock_client = Mock()
        mock_client.is_connected = True
        mock_ros.return_value = mock_client

        mock_service = Mock()
        mock_service.call.return_value = {'success': True}
        mock_service_class.return_value = mock_service

        helix = Helix("eai-helix-0.local")
        helix.connect()

        result = helix.switch_control_mode("position_control")

        assert result is True
        assert helix.get_control_mode() == "position_control"
        mock_service.call.assert_called_once()

    @patch('embodiedai_helix_api.helix.roslibpy.Ros')
    @patch('embodiedai_helix_api.helix.roslibpy.Service')
    def test_switch_control_mode_invalid(self, mock_service_class, mock_ros):
        """Test switching to invalid control mode."""
        mock_client = Mock()
        mock_client.is_connected = True
        mock_ros.return_value = mock_client

        helix = Helix("eai-helix-0.local")
        helix.connect()

        with pytest.raises(ValueError, match="Invalid control mode"):
            helix.switch_control_mode("invalid_mode")

    def test_switch_control_mode_not_connected(self):
        """Test switching control mode when not connected."""
        helix = Helix("eai-helix-0.local")

        with pytest.raises(ConnectionError, match="Not connected to robot"):
            helix.switch_control_mode("position_control")

    @patch('embodiedai_helix_api.helix.roslibpy.Ros')
    @patch('embodiedai_helix_api.helix.roslibpy.Service')
    def test_all_control_modes(self, mock_service_class, mock_ros):
        """Test all valid control modes."""
        mock_client = Mock()
        mock_client.is_connected = True
        mock_ros.return_value = mock_client

        mock_service = Mock()
        mock_service.call.return_value = {'success': True}
        mock_service_class.return_value = mock_service

        helix = Helix("eai-helix-0.local")
        helix.connect()

        valid_modes = ["none", "position_control", "current_control", "velocity_control"]

        for mode in valid_modes:
            result = helix.switch_control_mode(mode)
            assert result is True
            assert helix.get_control_mode() == mode


class TestControlModeEnum:
    """Test ControlMode enum."""

    def test_control_mode_values(self):
        """Test ControlMode enum values."""
        assert ControlMode.NONE.value == "none"
        assert ControlMode.POSITION_CONTROL.value == "position_control"
        assert ControlMode.CURRENT_CONTROL.value == "current_control"
        assert ControlMode.VELOCITY_CONTROL.value == "velocity_control"

    def test_control_mode_from_string(self):
        """Test creating ControlMode from string."""
        assert ControlMode("none") == ControlMode.NONE
        assert ControlMode("position_control") == ControlMode.POSITION_CONTROL
        assert ControlMode("current_control") == ControlMode.CURRENT_CONTROL
        assert ControlMode("velocity_control") == ControlMode.VELOCITY_CONTROL
