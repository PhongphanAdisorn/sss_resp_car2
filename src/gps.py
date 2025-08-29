import serial
import random
import time

class GPS:
    """
    Handles reading data from a GPS module.
    Can operate in two modes:
    1. Live Mode: Reads NMEA sentences from a specified serial port.
    2. File Mode: Reads sequentially from a list of data (from a file).
    """
    def __init__(self, port='/dev/ttyACM2', baud_rate=9600, from_file=None):
        """
        Initializes the GPS reader.
        :param port: The serial port to read from.
        :param baud_rate: The baud rate for the serial connection.
        :param from_file: Path to a sample file to read data from.
        """
        self.is_live = from_file is None
        self.data_lines = []
        self.line_index = 0

        if self.is_live:
            try:
                self.ser = serial.Serial(port, baud_rate, timeout=0)
                print(f"GPS listening on {port}.")
            except serial.SerialException:
                print(f"WARNING: Could not open serial port {port}. GPS will not provide data.")
                self.ser = None
        else:
            try:
                with open(from_file, 'r') as f:
                    self.data_lines = [line.strip() for line in f if line.strip()]
                print(f"GPS reading from sample file: {from_file}.")
            except FileNotFoundError:
                print(f"ERROR: Sample file not found: {from_file}.")
                self.data_lines = []

    def read_data(self):
        """
        Reads the next NMEA sentence from the source.
        In live mode, it reads a line from serial.
        In file mode, it reads the next line from the file.
        In simulated live mode (no serial), it generates a random GPGGA sentence.
        Returns the NMEA sentence as a string, or None if no data is available.
        """
        if self.is_live:
            if self.ser:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    return line if line else None
                return None
            else:
                # If serial failed but we are in live mode, simulate data
                lat = f"{random.uniform(13.7, 13.8):.6f}"
                lon = f"{random.uniform(100.5, 100.6):.6f}"
                return f"$GPGGA,123519.00,{lat},N,{lon},E,1,08,0.9,545.4,M,46.9,M,,*47"
        else:
            if self.line_index < len(self.data_lines):
                data = self.data_lines[self.line_index]
                self.line_index += 1
                return data
            return None

    def close(self):
        """Closes the serial connection if it's open."""
        if self.is_live and self.ser and self.ser.is_open:
            self.ser.close()
            print("GPS serial port closed.")

if __name__ == '__main__':
    # Example usage (requires a file 'gps_test.txt' in the same directory)
    print("--- GPS File Mode Test ---")
    with open('gps_test.txt', 'w') as f:
        f.write("$GPGGA,123519.00,1343.3250,N,10032.2217,E,1,08,0.9,545.4,M,46.9,M,,*47\n")
        f.write("$GPRMC,123519.00,A,1343.3250,N,10032.2217,E,022.4,084.4,230394,003.1,W*6A\n")

    gps = GPS(from_file='gps_test.txt')
    try:
        while True:
            data = gps.read_data()
            if data:
                print(f"Read GPS data: {data}")
            else:
                print("No more GPS data.")
                break
    finally:
        gps.close()
        import os
        os.remove('gps_test.txt')

    print("\n--- GPS Simulated Live Mode Test ---")
    gps_live = GPS()
    try:
        for _ in range(2):
            data = gps_live.read_data()
            print(f"Read simulated GPS data: {data}")
    finally:
        gps_live.close()

    print("--- GPS Test Complete ---")
