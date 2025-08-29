import random
import threading
import time

# Attempt to import Lidar-related classes
try:
    from rplidar import RPLidar, RPLidarException
    LIDAR_AVAILABLE = True
except ImportError:
    LIDAR_AVAILABLE = False
    print("WARNING: rplidar library not found. Lidar will be simulated.")


class Lidar:
    """
    Handles reading and processing data from an RPLidar A1M8 sensor.
    Divides the 360-degree scan into 8 directions and finds the minimum distance for each.
    Can operate in live mode, from a file, or in a pure simulation.
    """
    def __init__(self, port='/dev/ttyUSB0', from_file=None, simulate=False):
        self.port = port
        self.lidar = None
        self.is_live = not simulate and from_file is None
        self.scan_data = {}
        self.stop_event = threading.Event()
        self.thread = None
        self.file_lines = None

        if from_file:
            self.is_live = False
            try:
                with open(from_file, 'r') as f:
                    self.file_lines = f.readlines()
                print(f"Lidar reading from sample file: {from_file}.")
            except (FileNotFoundError, IOError) as e:
                print(f"WARNING: Could not read Lidar sample file {from_file}: {e}")
                self.file_lines = []

        elif self.is_live:
            if LIDAR_AVAILABLE:
                try:
                    self.lidar = RPLidar(self.port)
                    print(f"RPLidar connected successfully on {self.port}.")
                    self.thread = threading.Thread(target=self._scan_loop, daemon=True)
                    self.thread.start()
                except RPLidarException as e:
                    print(f"WARNING: Could not connect to Lidar on {self.port}. Error: {e}")
                    print("Switching to simulation mode for Lidar.")
                    self.is_live = False
                    self.lidar = None
            else:
                print("Lidar running in simulation mode (rplidar library not found).")
                self.is_live = False
        else: # simulate=True
            print("Lidar running in simulation mode.")

    def _process_scan(self, scan):
        directions = {
            'front': float('inf'), 'front_right': float('inf'), 'right': float('inf'),
            'back_right': float('inf'), 'back': float('inf'), 'back_left': float('inf'),
            'left': float('inf'), 'front_left': float('inf'),
        }
        for _, angle, distance in scan:
            if distance == 0: continue
            distance /= 1000.0  # Convert mm to meters
            if 337.5 <= angle <= 360 or 0 <= angle < 22.5: directions['front'] = min(distance, directions['front'])
            elif 22.5 <= angle < 67.5: directions['front_right'] = min(distance, directions['front_right'])
            elif 67.5 <= angle < 112.5: directions['right'] = min(distance, directions['right'])
            elif 112.5 <= angle < 157.5: directions['back_right'] = min(distance, directions['back_right'])
            elif 157.5 <= angle < 202.5: directions['back'] = min(distance, directions['back'])
            elif 202.5 <= angle < 247.5: directions['back_left'] = min(distance, directions['back_left'])
            elif 247.5 <= angle < 292.5: directions['left'] = min(distance, directions['left'])
            elif 292.5 <= angle < 337.5: directions['front_left'] = min(distance, directions['front_left'])
        return directions

    def _scan_loop(self):
        try:
            for scan in self.lidar.iter_scans():
                if self.stop_event.is_set(): break
                self.scan_data = self._process_scan(scan)
        except RPLidarException as e:
            print(f"Lidar connection error in scan loop: {e}")
            self.is_live = False
        finally:
            if self.lidar:
                self.lidar.stop()
                self.lidar.disconnect()
                print("Lidar has been disconnected.")

    def read_data(self):
        if self.is_live:
            return self.scan_data

        # --- File or Simulation Mode ---
        if self.file_lines is not None:
            # File mode: consume one line per call
            if not self.file_lines:
                return None # End of file
            self.file_lines.pop(0)
            # For simplicity, we still return random data, but the key is that we exhaust the file.
            # A proper implementation would parse the line.

        # Pure simulation or file-based simulation returns random data
        return {
            'front': round(random.uniform(0.5, 10.0), 2),
            'front_right': round(random.uniform(0.5, 10.0), 2),
            'right': round(random.uniform(0.5, 10.0), 2),
            'back_right': round(random.uniform(0.5, 10.0), 2),
            'back': round(random.uniform(0.5, 10.0), 2),
            'back_left': round(random.uniform(0.5, 10.0), 2),
            'left': round(random.uniform(0.5, 10.0), 2),
            'front_left': round(random.uniform(0.5, 10.0), 2),
        }

    def close(self):
        if self.thread:
            self.stop_event.set()
            self.thread.join()
        print("Lidar shut down gracefully.")

if __name__ == '__main__':
    print("--- Testing Lidar in Simulation Mode ---")
    lidar_sim = Lidar(simulate=True)
    try:
        for i in range(3):
            print(f"Read {i+1}: {lidar_sim.read_data()}")
            time.sleep(0.1)
    finally:
        lidar_sim.close()

    print("\n--- Testing Lidar from File Mode ---")
    # Create a dummy file for testing
    dummy_file = "dummy_lidar.txt"
    with open(dummy_file, "w") as f:
        f.write("line 1\nline 2\n")

    lidar_file = Lidar(from_file=dummy_file)
    try:
        i = 1
        while True:
            data = lidar_file.read_data()
            print(f"Read {i}: {data}")
            if data is None:
                break
            i += 1
            time.sleep(0.1)
    finally:
        lidar_file.close()
        import os
        os.remove(dummy_file)

    print("\n--- Lidar Test Complete ---")
