import time
import serial
import struct
import logging

# --- New IBusReader Class provided by user ---
IBUS_FRAME_LEN = 32
IBUS_HEADER0 = 0x20
IBUS_HEADER1 = 0x40

class IBusReader:
    def __init__(self, port="/dev/serial0", baudrate=115200, timeout=0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.last_frame_ts = 0.0
        self.last_frame = None
        # Modified to include channels for compatibility
        self.last_controls = {"steering": 0.0, "throttle": 0.0, "mode": "manual", "channels": []}
        self.frame_width = 0.01  # 1ms frame
        self.min_val = 1000e-6   # 1.0 ms
        self.mid_val = 1500e-6   # 1.5 ms
        self.max_val = 2000e-6   # 2.0 ms
        self.offset = 30e-6      # offset 30 ยตs
        self.max_speed = 1.0     # limit max throttle
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            # Using print for now as no logger is passed
            print(f"? Connected to {self.port}")
        except Exception as e:
            print(f"? Connect failed: {e}")
            self.ser = None

    def reconnect_if_needed(self):
        if self.ser is None or not self.ser.is_open:
            self.connect()

    def _read_exact(self, n):
        if self.ser is None:
            return None
        try:
            data = self.ser.read(n)
            return data if len(data) == n else None
        except Exception:
            print("?? Serial read error, reconnecting...")
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
            return None

    def _read_frame(self):
        self.reconnect_if_needed()
        if self.ser is None:
            return None

        b = self._read_exact(1)
        if not b or b[0] != IBUS_HEADER0:
            return None
        b2 = self._read_exact(1)
        if not b2 or b2[0] != IBUS_HEADER1:
            return None
        rest = self._read_exact(IBUS_FRAME_LEN - 2)
        if not rest:
            return None
        frame = bytes([IBUS_HEADER0, IBUS_HEADER1]) + rest
        return frame if self._valid_checksum(frame) else None

    @staticmethod
    def _valid_checksum(frame: bytes) -> bool:
        if len(frame) != IBUS_FRAME_LEN:
            return False
        checksum = 0xFFFF
        for b in frame[:30]:
            checksum = (checksum - b) & 0xFFFF
        rx_chk = frame[30] | (frame[31] << 8)
        return checksum == rx_chk

    @staticmethod
    def _parse_channels(frame: bytes):
        ch = []
        for i in range(14):
            lo = frame[2 + i*2]
            hi = frame[3 + i*2]
            ch.append((hi << 8) | lo)
        return ch

    def read_channels(self):
        frame = self._read_frame()
        if frame is None:
            return None
        self.last_frame = frame
        self.last_frame_ts = time.time()
        return self._parse_channels(frame)

    def get_controls(self, offset=0.002):
        channels = self.read_channels()

        # If no new channels are read, check for signal loss
        if not channels:
            # If it's been too long since the last good frame, return None
            # to indicate signal loss. A grace period prevents jerky behavior
            # from transient read failures.
            #if self.last_frame_ts > 0 and (time.time() - self.last_frame_ts) > 0.5: # 500ms timeout
            #    return None

            # Within the grace period, return the last known controls.
            if self.last_frame:
                self.last_controls['raw_frame'] = self.last_frame
            return self.last_controls

        # --- CH1 steering ---
        steering = (channels[0] - 1500) / 500.0
        steering = max(-1.0, min(1.0, steering))

        # --- CH2 throttle ---
        throttle = (channels[1] - 1500) / 500.0
        throttle = max(-1.0, min(1.0, throttle))

        # --- CH5 mode ---
        ch5_val = channels[4]  # CH5 index = 4
        if ch5_val < 1400:
            mode = "manual"
        elif ch5_val > 1600:
            mode = "auto"
        else:
            # If in the middle, retain the last mode
            mode = self.last_controls["mode"]

        # --- offset filter ---
        if abs(steering - self.last_controls["steering"]) <= offset:
            steering = self.last_controls["steering"]
        if abs(throttle - self.last_controls["throttle"]) <= offset:
            throttle = self.last_controls["throttle"]

        self.last_controls = {
            "steering": steering,
            "throttle": throttle,
            "mode": mode,
            "channels": channels, # Modified to include channels for compatibility
            "raw_frame": self.last_frame
        }
        return self.last_controls

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except:
                pass

# --- Constants for original file-based reader ---
IBUS_HEADER_OLD = b'\x20\x40'
IBUS_PACKET_LENGTH = 32
IBUS_CHANNELS = 14

CHANNEL_MAP = {
    'steering': 0,
    'throttle': 1,
    'mode': 4,
}
CHANNEL_MIN = 1000
CHANNEL_MAX = 2000
CHANNEL_CENTER = 1500
CHANNEL_DEADZONE = 50
MODE_SWITCH_LOW = 1400
MODE_SWITCH_MID = 1800


# --- Modified RemoteControl Class (Wrapper) ---
class RemoteControl:
    """
    Handles reading and parsing data from a FlySky I-Bus receiver.
    Provides normalized values for steering and throttle, and handles mode switching.
    This class wraps the new IBusReader for live operation and uses the original
    file-reading logic for simulation/testing.
    """
    def __init__(self, port='/dev/ttyAMA2', baud_rate=115200, from_file=None, logger=None):
        self.port = port
        self.baud_rate = baud_rate
        self.is_live = from_file is None
        self.logger = logger or logging.getLogger(__name__)
        self.file_data = None
        self.ibus_reader = None

        # Mode control for file-based reader
        self.modes = ["manual", "auto", "manual"]
        self.current_mode_index = 0
        self.current_mode = self.modes[self.current_mode_index]

        if self.is_live:
            self.logger.info("Remote Control running in LIVE mode.")
            self.ibus_reader = IBusReader(port=self.port, baudrate=self.baud_rate)
        else:
            try:
                with open(from_file, 'rb') as f:
                    self.file_data = f.read()
                self.logger.info(f"Remote Control reading from sample file: {from_file}.")
            except (FileNotFoundError, IOError) as e:
                self.logger.error(f"Could not read sample file {from_file}: {e}")
                self.file_data = b''

    def read_data(self):
        """
        Reads data from the live I-Bus reader or from a file.
        Returns a dictionary with control data or None.
        """
        if self.is_live:
            if self.ibus_reader:
                return self.ibus_reader.get_controls()
            return None

        # --- File mode logic (preserved from original) ---
        packet = None
        if self.file_data and len(self.file_data) >= IBUS_PACKET_LENGTH:
            # Check for header before consuming data
            if self.file_data.startswith(IBUS_HEADER_OLD):
                packet = self.file_data[:IBUS_PACKET_LENGTH]
                self.file_data = self.file_data[IBUS_PACKET_LENGTH:]
            else:
                # If no header, maybe the file is misaligned. For now, we stop.
                self.logger.warning("IBus header not found at start of file data.")
                self.file_data = b'' # Clear buffer to prevent infinite loops
                return None

        if packet:
            if self._validate_checksum(packet):
                channel_values = struct.unpack('<' + 'H' * IBUS_CHANNELS, packet[2:30])
                self.logger.debug(f"IBus Raw Channels (from file): {channel_values}")
                result = self._get_control_values(list(channel_values))
                result['raw_frame'] = packet
                return result
            else:
                self.logger.warning("IBus checksum mismatch (from file).")

        return None

    def close(self):
        """Closes the connection/resources."""
        if self.is_live and self.ibus_reader:
            self.ibus_reader.close()
            self.logger.info("I-Bus reader resources released.")

    # --- Methods for file-based reading (preserved from original) ---
    def _validate_checksum(self, packet):
        if len(packet) != IBUS_PACKET_LENGTH:
            return False
        calculated_checksum = sum(packet[:-2])
        expected_checksum = struct.unpack('<H', packet[30:32])[0]
        return (0xFFFF - calculated_checksum) == expected_checksum

    def _normalize_channel_value(self, value):
        if abs(value - CHANNEL_CENTER) < CHANNEL_DEADZONE:
            return 0.0
        if value < CHANNEL_CENTER:
            return (value - CHANNEL_CENTER) / (CHANNEL_CENTER - CHANNEL_MIN)
        else:
            return (value - CHANNEL_CENTER) / (CHANNEL_MAX - CHANNEL_CENTER)

    def _get_control_values(self, channels):
        mode_val = channels[CHANNEL_MAP['mode']]
        new_mode_index = self.current_mode_index
        if mode_val < MODE_SWITCH_LOW:
            new_mode_index = 0
        elif mode_val < MODE_SWITCH_MID:
            new_mode_index = 1
        else:
            new_mode_index = 2

        if new_mode_index != self.current_mode_index:
            self.current_mode_index = new_mode_index
            self.current_mode = self.modes[self.current_mode_index]
            self.logger.info(f"Mode changed to '{self.current_mode}' (index: {self.current_mode_index}).")

        throttle = self._normalize_channel_value(channels[CHANNEL_MAP['throttle']])
        steering = self._normalize_channel_value(channels[CHANNEL_MAP['steering']])

        return {
            'mode': self.current_mode,
            'throttle': throttle,
            'steering': steering,
            'channels': channels,
        }
