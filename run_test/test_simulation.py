import unittest
import logging
from unittest.mock import patch

# Import the classes to be tested
from src.motor import Motor
from src.lidar import Lidar
from src.keyboard_control import KeyboardControl

# Disable logging for cleaner test output
logging.disable(logging.CRITICAL)

# A mock event class to simulate keyboard events
class MockEvent:
    def __init__(self, name):
        self.name = name

class TestSimulationMode(unittest.TestCase):

    @patch('keyboard.unhook_all')
    @patch('keyboard.on_press')
    def test_keyboard_control(self, mock_on_press, mock_unhook_all):
        """Test if KeyboardControl correctly updates its state."""
        kb = KeyboardControl()

        mock_on_press.assert_called_once()

        # Simulate pressing 'w' (throttle up)
        kb._on_press(MockEvent('w'))
        data = kb.read_data()
        self.assertIsNotNone(data)
        self.assertAlmostEqual(data['throttle'], 0.1)
        self.assertAlmostEqual(data['steering'], 0.0)

        # Simulate pressing 'd' (steering right)
        kb._on_press(MockEvent('d'))
        data = kb.read_data()
        self.assertIsNotNone(data)
        self.assertAlmostEqual(data['throttle'], 0.1)
        self.assertAlmostEqual(data['steering'], 0.1)

        kb.close()
        mock_unhook_all.assert_called_once()

    def test_simulated_lidar(self):
        """Test if the Lidar in simulation mode provides valid data."""
        lidar = Lidar(simulate=True)
        data = lidar.read_data()

        self.assertIsNotNone(data)
        self.assertIsInstance(data, dict)

        expected_keys = ['front', 'front_right', 'right', 'back_right', 'back', 'back_left', 'left', 'front_left']
        self.assertCountEqual(data.keys(), expected_keys)

        for key, value in data.items():
            self.assertIsInstance(value, float)

        lidar.close()

    @patch('keyboard.unhook_all')
    @patch('keyboard.on_press')
    def test_simulation_loop_integration(self, mock_on_press, mock_unhook_all):
        """Test the integration between keyboard control and motor in simulation."""
        kb = KeyboardControl()
        motor = Motor()

        # Simulate forward-right command
        kb._on_press(MockEvent('w')) # throttle = 0.1
        kb._on_press(MockEvent('w')) # throttle = 0.2
        kb._on_press(MockEvent('d')) # steering = 0.1

        control_data = kb.read_data()
        self.assertIsNotNone(control_data)

        # Pass control data to motor
        motor.move(control_data['throttle'], control_data['steering'])

        self.assertAlmostEqual(motor.left_speed, 0.3)
        self.assertAlmostEqual(motor.right_speed, 0.1)

        motor.cleanup()
        kb.close()
        mock_unhook_all.assert_called_once()

if __name__ == '__main__':
    unittest.main()
