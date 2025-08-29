# Raspberry Pi Sensor and Motor Control Project

This project provides a complete framework for controlling a PWM motor and reading data from multiple sensors (Remote Control, Lidar, GPS) on a Raspberry Pi. It includes features for data logging, simulation for development on non-Pi systems, and sample data for testing. The system is optimized for real-time performance by removing all `time.sleep()` calls.

## Project Structure

```
.
├── Makefile              # Automation for installing, running, and cleaning
├── README.md             # This file
├── requirements.txt      # Python dependencies
├── src/                  # Main Python source code
│   ├── __init__.py
│   ├── gps.py
│   ├── lidar.py
│   ├── main.py
│   ├── motor.py
│   └── remote.py
├── test_remote.py        # Unit tests for the remote control module
├── logs/                 # Directory for sensor log files (created automatically)
└── example/              # Example data and scripts
    ├── README.md
    ├── example_data_generator.py
    ├── gps_sample.txt
    ├── lidar_sample.txt
    └── remote_control_sample.txt
```

## Installation

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```

2.  **Install dependencies:**
    A `Makefile` is provided to simplify installation.

    ```bash
    make install
    ```

    This will run `pip install -r requirements.txt`.

    **Note for non-Raspberry Pi systems:** The `gpiozero` library will fail to install without the underlying `RPi.GPIO` dependency (which is specific to Raspberry Pi OS). You must comment it out in `requirements.txt` before running `make install`. The scripts are designed to work in a simulated mode when this library is not present.

## How to Run

### 1. Running with Sample Data

This is the recommended way to test the application's logic on any machine (including a standard Linux VM). It reads from the pre-generated files in the `example/` directory.

```bash
make run-sample
```

This command will:
1.  Generate fresh sample data using `example/example_data_generator.py`.
2.  Run the main script `src/main.py` using the sample data as input.
3.  You will see simulated motor actions printed to the console and log files created in the `logs/` directory.

### 2. Running in Simulation Mode

This mode allows you to control the robot using your keyboard. It's great for testing control logic without needing any hardware.

```bash
make run-simulate
```

Or directly:
```bash
python src/main.py --simulate
```

**Controls:**
-   **Up Arrow:** Increase speed by 1.0.
-   **Down Arrow:** Decrease speed by 1.0.
-   **Left Arrow:** Decrease angle by 0.5 degrees.
-   **Right Arrow:** Increase angle by 0.5 degrees.

The simulation will display the current speed and angle, along with **simulated 8-direction Lidar data**, in the console. All of this data is saved to `logs/simulation.log`.

### 3. Running in Live Mode (Raspberry Pi Only)

This mode attempts to connect to real hardware sensors on the specified serial ports.

```bash
python src/main.py --live
```

The script will try to read from the default hardware ports:
-   **Remote Control (I-Bus):** `/dev/ttyAMA2`
-   **Lidar Sensor:** `/dev/ttyACM1`
-   **GPS Sensor:** `/dev/ttyACM2`

And it will control the motor using the L298N driver connected to GPIO pins.

### Remote Control (I-Bus Protocol)
The system is configured to read from a **FlySky FS-iA10B receiver** using the **I-Bus protocol**.
- **Connection:** The receiver's I-Bus output should be connected to the Raspberry Pi's UART2 RX pin (GPIO5), which corresponds to the `/dev/ttyAMA2` device path.
- **Protocol:** The communication uses a baud rate of `115200`. The `remote.py` script automatically handles I-Bus packet parsing, checksum validation, and error logging.
- **Reconnection:** If the connection to the receiver is lost, the system will automatically attempt to reconnect.

## Lidar Data Processing (RPLidar A1M8)

The system interfaces with an RPLidar A1M8 sensor to provide a 360-degree view of the environment. The raw data from the Lidar is processed into a more usable, high-level format.

### 8-Directional Sensing
To simplify obstacle avoidance and navigation logic, the 360-degree scan is divided into **8 primary directions**:
- `front`
- `front_right`
- `right`
- `back_right`
- `back`
- `back_left`
- `left`
- `front_left`

Each direction covers a 45-degree cone. For each cone, the system calculates the **minimum distance** to any detected obstacle, providing a simple and effective way to gauge proximity. This processed data is logged to `logs/lidar.log` and displayed in the console.

### Connection Schematic (Raspberry Pi & Lidar)

The RPLidar A1M8 connects to the Raspberry Pi via USB for data and requires a separate 5V power supply.

```
+------------------+      +----------------------+
| Raspberry Pi 4   |      |   RPLidar A1M8       |
|                  |      |                      |
|       USB Port <----(Data)---> USB Port        |
|                  |      |                      |
|       GPIO 5V    |----(Power)--> 5V (M+)       |
|       GPIO GND   |----(Ground)--> GND (M-)     |
+------------------+      +----------------------+
      |      ^
      |      |
(Power) V      | (Optional: Can be powered from Pi's GPIO)
      |      |
+---------------------+
| External 5V Supply  |
+---------------------+
```

- **Data Connection:** The Lidar's USB cable plugs directly into one of the Raspberry Pi's USB ports. This connection is recognized as `/dev/ttyUSB0` by the OS.
- **Power Connection:** The Lidar requires a stable 5V power source. While it can be powered from the Pi's GPIO pins, an external power supply is recommended for stability.

## Motor Control with PWM (ESC and Servo)

The system now uses `gpiozero.PWMOutputDevice` to provide precise PWM signals for controlling both the main drive motor (via an Electronic Speed Controller, ESC) and the steering servo. This setup is common in RC cars and other robotic vehicles.

- **Throttle Control (ESC):** Connected to **GPIO 18**.
- **Steering Control (Servo):** Connected to **GPIO 19**.

### PWM Configuration
The PWM signals for both devices are configured as follows:
- **Frame Width:** 20ms (50Hz frequency), the standard for most ESCs and servos.
- **Pulse Width Range:**
  - **Minimum (1.0ms):** Corresponds to full reverse (throttle) or full left (steering).
  - **Mid-point (1.5ms):** Corresponds to neutral (throttle) or center (steering).
  - **Maximum (2.0ms):** Corresponds to full forward (throttle) or full right (steering).

The `motor.py` script handles the mapping of normalized control values (`-1.0` to `1.0`) from the remote control to this pulse width range.

### Connection Schematic (Raspberry Pi, ESC, and Servo)

It is critical to power the ESC and servo from a dedicated power source (like a BEC or separate battery) and NOT directly from the Raspberry Pi's 5V pin, as they can draw significant current and cause the Pi to become unstable.

```
+------------------+      +------------------+      +------------------+
| Raspberry Pi 4   |      |       ESC        |      |   Steering Servo |
|                  |      | (Motor Driver)   |      |                  |
|          GPIO 18 |----(Signal)--> PWM Input|      |                  |
|          GPIO 19 |-----------------------------(Signal)--> PWM Input|
|           Ground |----(Ground Ref)----------------(Ground Ref)------|
+------------------+      +------------------+      +------------------+
                            |          ^                |
                            | (Power)  | (To Motor)     | (Power)
                            |          |                |
                       +----V----------V----+      +----V----+
                       |   BEC / Battery   |      |  (Same) |
                       +-------------------+      +---------+
```
- **Signal Wires:** The signal pins of the ESC and servo connect to GPIO 18 and 19 respectively.
- **Ground Wires:** The ground wires of the ESC and servo **must** be connected to one of the Raspberry Pi's ground pins to ensure a common ground reference.
- **Power Wires:** The positive (red) power wires for the ESC and servo should be connected to a suitable external power source.

## Coordinate Reference System (CRS) Note

The GPS module reads and logs standard NMEA sentences. It does not perform any Coordinate Reference System (CRS) transformations. The logged latitude and longitude data is based on the WGS 84 datum, which is the standard for GPS. If your application requires a different CRS (e.g., for projecting onto a 2D map), you will need to implement the appropriate conversion logic.

## I-Bus Remote Control Simulation

The remote control module (`src/remote.py`) simulates a FlySky FS-iA10B receiver and provides advanced control features based on the I-Bus protocol.

### Control Logic
- **Normalized Values**: Instead of discrete commands, the remote module provides normalized floating-point values for throttle and steering, ranging from **-1.0 to 1.0**.
  - **Throttle**: -1.0 (full reverse) to 1.0 (full forward), with 0.0 being neutral.
  - **Steering**: -1.0 (full left) to 1.0 (full right), with 0.0 being center.
- **Deadzone**: A small deadzone around the center stick positions is implemented to prevent drift.

### Channel Mapping
The simulation uses the following channels for control:
- **Channel 1 (CH1)**: Steering (Left/Right)
- **Channel 2 (CH2)**: Throttle (Forward/Backward)
- **Channel 5 (CH5)**: Mode Selection

### Mode Selection
Channel 5 is used to switch between different operational modes using a 3-position switch on the remote controller. The mode is determined by the raw channel value, which is divided into three distinct ranges:

- **Position 1 (Low):** `1000-1400` - Activates the first mode (`manual`).
- **Position 2 (Middle):** `1401-1800` - Activates the second mode (`auto`).
- **Position 3 (High):** `1801-2000` - Activates the third mode (`manual`).

The system logs a message every time the mode changes.

### Real-time Data Monitoring
For debugging and real-time monitoring, the console output in `--live` or sample-file mode now displays the raw values for all 14 I-Bus channels, in addition to the normalized throttle, steering, and current mode. This allows for a detailed view of the data being received from the remote controller. The full data, including the channel list, is also saved to `logs/remote.log`.
