import unittest
import logging
import serial
from unittest.mock import patch, MagicMock

# Import the classes to be tested
from src.motor import Motor
from src.lidar import Lidar
from src.gps import GPS
from src.remote import RemoteControl, IBusReader

# Disable logging for cleaner test output
logging.disable(logging.CRITICAL)

class TestMotor(unittest.TestCase):
    def setUp(self):
        self.motor = Motor()

    def test_move_forward(self):
        self.motor.move(throttle=0.8, steering=0)
        self.assertAlmostEqual(self.motor.left_speed, 0.8)
        self.assertAlmostEqual(self.motor.right_speed, 0.8)

    def test_move_backward(self):
        self.motor.move(throttle=-0.5, steering=0)
        self.assertAlmostEqual(self.motor.left_speed, -0.5)
        self.assertAlmostEqual(self.motor.right_speed, -0.5)

    def test_turn_right(self):
        self.motor.move(throttle=0, steering=1.0)
        self.assertAlmostEqual(self.motor.left_speed, 1.0)
        self.assertAlmostEqual(self.motor.right_speed, -1.0)

    def test_turn_left(self):
        self.motor.move(throttle=0, steering=-1.0)
        self.assertAlmostEqual(self.motor.left_speed, -1.0)
        self.assertAlmostEqual(self.motor.right_speed, 1.0)

    def test_forward_left(self):
        self.motor.move(throttle=0.6, steering=-0.4)
        self.assertAlmostEqual(self.motor.left_speed, 0.2) # 0.6 - 0.4
        self.assertAlmostEqual(self.motor.right_speed, 1.0) # 0.6 + 0.4

    def test_clamping(self):
        # Test that values are clamped to [-1, 1] range
        self.motor.move(throttle=1.0, steering=0.5)
        self.assertAlmostEqual(self.motor.left_speed, 1.0) # 1.0 + 0.5 = 1.5 -> clamped to 1.0
        self.assertAlmostEqual(self.motor.right_speed, 0.5) # 1.0 - 0.5

    def tearDown(self):
        self.motor.cleanup()


class TestLidar(unittest.TestCase):
    def setUp(self):
        # We don't need a real Lidar for this, just the class instance
        self.lidar = Lidar(simulate=True)

    def test_process_scan_directions(self):
        """Test the logic that processes a raw scan into 8 directions."""
        # Create a sample scan (quality, angle, distance)
        # Distances are in mm
        scan_data = [
            (10, 0, 1000),    # Front
            (10, 15, 2000),   # Front
            (10, 45, 3000),   # Front-Right
            (10, 90, 4000),   # Right
            (10, 135, 5000),  # Back-Right
            (10, 180, 6000),  # Back
            (10, 225, 7000),  # Back-Left
            (10, 270, 8000),  # Left
            (10, 315, 9000),  # Front-Left
            (10, 350, 500),   # Front (closer)
        ]

        processed = self.lidar._process_scan(scan_data)

        # The function should find the minimum distance in each cone (in meters)
        self.assertAlmostEqual(processed['front'], 0.5) # 500mm
        self.assertAlmostEqual(processed['front_right'], 3.0) # 3000mm
        self.assertAlmostEqual(processed['right'], 4.0)
        self.assertAlmostEqual(processed['back_right'], 5.0)
        self.assertAlmostEqual(processed['back'], 6.0)
        self.assertAlmostEqual(processed['back_left'], 7.0)
        self.assertAlmostEqual(processed['left'], 8.0)
        self.assertAlmostEqual(processed['front_left'], 9.0)

    def test_process_scan_empty_and_infinity(self):
        """Test with an empty scan and ensure unassigned directions are infinity."""
        processed = self.lidar._process_scan([])
        # All directions should remain at their initial 'inf' value
        for direction, distance in processed.items():
            self.assertEqual(distance, float('inf'))

    def tearDown(self):
        self.lidar.close()


class TestGPS(unittest.TestCase):
    def setUp(self):
        self.test_file = 'test_gps_data.txt'
        with open(self.test_file, 'w') as f:
            f.write("$GPGGA,123519.00,1343.3250,N,10032.2217,E,1,08,0.9,545.4,M,46.9,M,,*47\n")
            f.write("$GPRMC,123519.00,A,1343.3250,N,10032.2217,E,022.4,084.4,230394,003.1,W*6A\n")

    def test_read_from_file(self):
        """Test reading GPS data sequentially from a file."""
        gps = GPS(from_file=self.test_file)
        line1 = gps.read_data()
        self.assertEqual(line1, "$GPGGA,123519.00,1343.3250,N,10032.2217,E,1,08,0.9,545.4,M,46.9,M,,*47")
        line2 = gps.read_data()
        self.assertEqual(line2, "$GPRMC,123519.00,A,1343.3250,N,10032.2217,E,022.4,084.4,230394,003.1,W*6A")
        # Test end of file
        self.assertIsNone(gps.read_data())
        gps.close()

    @patch('serial.Serial')
    def test_simulated_live_mode(self, mock_serial):
        """Test the fallback to simulated data when serial fails in live mode."""
        # Make the serial constructor fail
        mock_serial.side_effect = serial.SerialException("Port not found")

        # The GPS class should catch the exception and switch to simulation
        gps = GPS(port='/dev/nonexistent')
        self.assertIsNone(gps.ser) # ser should be None

        # read_data should now return a simulated GPGGA string
        data = gps.read_data()
        self.assertIsNotNone(data)
        self.assertTrue(data.startswith('$GPGGA'))
        gps.close()

    def tearDown(self):
        import os
        if os.path.exists(self.test_file):
            os.remove(self.test_file)


class TestIBusReader(unittest.TestCase):

    def _create_ibus_frame(self, channels):
        """Creates a valid I-Bus frame for the new IBusReader checksum."""
        if len(channels) != 14:
            raise ValueError("Must be 14 channels.")

        frame = b'\x20\x40' # Header
        for ch_val in channels:
            frame += ch_val.to_bytes(2, 'little')

        checksum = 0xFFFF
        for byte in frame:
            checksum -= byte

        frame += checksum.to_bytes(2, 'little')
        return frame

    @patch('serial.Serial')
    def test_get_controls_normalization(self, mock_serial_class):
        """Test the control normalization of IBusReader."""
        # Mock the serial port instance and its read method
        mock_serial_instance = MagicMock()
        mock_serial_class.return_value = mock_serial_instance

        # --- Test Case 1: Centered sticks ---
        channels_center = [1500] * 14
        frame_center = self._create_ibus_frame(channels_center)

        # Configure the mock to return chunks as _read_exact expects
        mock_serial_instance.read.side_effect = [
            frame_center[0:1],
            frame_center[1:2],
            frame_center[2:]
        ]

        reader = IBusReader()
        controls = reader.get_controls()

        self.assertAlmostEqual(controls['steering'], 0.0)
        self.assertAlmostEqual(controls['throttle'], 0.0)
        self.assertEqual(controls['mode'], 'manual') # Default initial mode

        # --- Test Case 2: Full throttle, half right, auto mode ---
        channels_test = [1750, 2000, 1500, 1500, 1800, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        frame_test = self._create_ibus_frame(channels_test)
        mock_serial_instance.read.side_effect = [
            frame_test[0:1],
            frame_test[1:2],
            frame_test[2:]
        ]

        controls = reader.get_controls()
        self.assertAlmostEqual(controls['steering'], 0.5) # (1750-1500)/500
        self.assertAlmostEqual(controls['throttle'], 1.0) # (2000-1500)/500
        self.assertEqual(controls['mode'], 'auto')

    def test_checksum_validation(self):
        """Test the static checksum validation method of IBusReader."""
        channels = [1500] * 14
        good_frame = self._create_ibus_frame(channels)

        bad_frame_list = bytearray(good_frame)
        bad_frame_list[10] += 1 # Corrupt a data byte
        bad_frame = bytes(bad_frame_list)

        self.assertTrue(IBusReader._valid_checksum(good_frame))
        self.assertFalse(IBusReader._valid_checksum(bad_frame))


if __name__ == '__main__':
    # We need to import serial for the mock to work on SerialException
    import serial
    unittest.main()
