import unittest
import struct
import os
import logging
from src.remote import RemoteControl, IBUS_PACKET_LENGTH, IBUS_HEADER_OLD as IBUS_HEADER

# Disable logging for tests to keep the output clean
logging.disable(logging.CRITICAL)

def create_ibus_packet(channels):
    """
    Helper function to create a valid I-Bus packet from a list of 14 channel values.
    """
    if len(channels) != 14:
        raise ValueError("Channel list must contain exactly 14 values.")

    # Start with the header
    packet = IBUS_HEADER

    # Add channel data (little-endian unsigned short)
    packet += struct.pack('<' + 'H' * 14, *channels)

    # Calculate and add checksum
    checksum = 0xFFFF - sum(packet)
    packet += struct.pack('<H', checksum)

    return packet

class TestRemoteControl(unittest.TestCase):

    def setUp(self):
        """Set up some default channel values for tests."""
        self.channels_stop = [1500] * 14
        self.test_file = 'test_ibus_data.bin'

    def tearDown(self):
        """Clean up any files created during tests."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)

    def test_packet_creation(self):
        """Test the helper function to ensure it creates valid packets."""
        packet = create_ibus_packet(self.channels_stop)
        self.assertEqual(len(packet), IBUS_PACKET_LENGTH)
        self.assertTrue(packet.startswith(IBUS_HEADER))

        # Manually verify checksum
        checksum_val = struct.unpack('<H', packet[30:32])[0]
        self.assertEqual(0xFFFF - sum(packet[:-2]), checksum_val)

    def test_checksum_validation(self):
        """Test the checksum validation logic."""
        # Create a valid packet
        valid_packet = create_ibus_packet(self.channels_stop)

        # Create an invalid packet by corrupting a byte
        invalid_packet_list = list(valid_packet)
        invalid_packet_list[10] = 0x11 # Corrupt a data byte
        invalid_packet = bytes(invalid_packet_list)

        remote = RemoteControl(from_file=self.test_file)
        self.assertTrue(remote._validate_checksum(valid_packet))
        self.assertFalse(remote._validate_checksum(invalid_packet))

    def test_read_multiple_commands_from_file(self):
        """Test reading a sequence of commands from a file."""
        # This test is now obsolete due to the change in return type,
        # but we can adapt it to test the new `read_data` method.
        # Create packets for right steering then backward throttle
        ch_right = [1500] * 14
        ch_right[0] = 1800 # Steering right
        packet1 = create_ibus_packet(ch_right)

        ch_backward = [1500] * 14
        ch_backward[1] = 1200 # Throttle backward
        packet2 = create_ibus_packet(ch_backward)

        with open(self.test_file, 'wb') as f:
            f.write(packet1 + packet2)

        remote = RemoteControl(from_file=self.test_file)
        data1 = remote.read_data()
        data2 = remote.read_data()

        self.assertIsNotNone(data1)
        self.assertAlmostEqual(data1['steering'], 0.6) # (1800-1500)/500
        self.assertAlmostEqual(data1['throttle'], 0.0)

        self.assertIsNotNone(data2)
        self.assertAlmostEqual(data2['steering'], 0.0)
        self.assertAlmostEqual(data2['throttle'], -0.6) # (1200-1500)/500

        self.assertIsNone(remote.read_data())

    def test_get_control_values(self):
        """Test the logic for normalization and inclusion of raw channels."""
        # Test case 1: Centered sticks
        channels = [1500] * 14
        packet = create_ibus_packet(channels)
        with open(self.test_file, 'wb') as f: f.write(packet)
        remote = RemoteControl(from_file=self.test_file)
        data = remote.read_data()
        self.assertAlmostEqual(data['throttle'], 0.0)
        self.assertAlmostEqual(data['steering'], 0.0)
        self.assertIn('channels', data)
        self.assertEqual(len(data['channels']), 14)

        # Test case 2: Full throttle forward, half right
        channels[1] = 2000 # ch2 throttle
        channels[0] = 1750 # ch1 steering
        packet = create_ibus_packet(channels)
        with open(self.test_file, 'wb') as f: f.write(packet)
        remote = RemoteControl(from_file=self.test_file)
        data = remote.read_data()
        self.assertAlmostEqual(data['throttle'], 1.0)
        self.assertAlmostEqual(data['steering'], 0.5)

    def test_mode_switching(self):
        """Test the 3-position mode switching logic."""
        channels = [1500] * 14
        remote = RemoteControl(from_file=self.test_file)

        # Test position 1 (Low) -> manual
        channels[4] = 1100
        packet = create_ibus_packet(channels)
        with open(self.test_file, 'wb') as f: f.write(packet)
        remote = RemoteControl(from_file=self.test_file)
        data = remote.read_data()
        self.assertEqual(data['mode'], 'manual')

        # Test position 2 (Mid) -> auto
        channels[4] = 1600
        packet = create_ibus_packet(channels)
        with open(self.test_file, 'wb') as f: f.write(packet)
        remote = RemoteControl(from_file=self.test_file)
        data = remote.read_data()
        self.assertEqual(data['mode'], 'auto')

        # Test position 3 (High) -> manual
        channels[4] = 1900
        packet = create_ibus_packet(channels)
        with open(self.test_file, 'wb') as f: f.write(packet)
        remote = RemoteControl(from_file=self.test_file)
        data = remote.read_data()
        self.assertEqual(data['mode'], 'manual')

    def test_raw_frame_in_output(self):
        """Test that the raw frame is included in the output data."""
        packet = create_ibus_packet(self.channels_stop)
        with open(self.test_file, 'wb') as f:
            f.write(packet)

        remote = RemoteControl(from_file=self.test_file)
        data = remote.read_data()

        self.assertIn('raw_frame', data)
        self.assertEqual(data['raw_frame'], packet)

if __name__ == '__main__':
    unittest.main()
