import argparse
import time
import os
import logging
from datetime import datetime

from motor import Motor
from remote import RemoteControl
from lidar import Lidar
from gps import GPS

def setup_logger(name, log_file, level=logging.INFO):
    """Function to setup as many loggers as you want"""
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    handler = logging.FileHandler(log_file, mode='w')
    handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s'))

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger

def main():
    """Main function to run the robot's control loop."""
    parser = argparse.ArgumentParser(description="Raspberry Pi Robot Control System")

    # Mode selection
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('--live', action='store_true', help="Run in live mode, connecting to real hardware.")
    mode_group.add_argument('--simulate', action='store_true', help="Run in simulation mode, using keyboard for control.")
    mode_group.add_argument('--input-remote', dest='input_remote', help="Run in file mode, using sample data for the remote.")

    # Additional inputs for file mode
    parser.add_argument('--input-lidar', dest='input_lidar', help="Sample data file for Lidar.")
    parser.add_argument('--input-gps', dest='input_gps', help="Sample data file for GPS.")

    # New flag for showing raw remote data
    parser.add_argument('--show-remote-raw-data', action='store_true', help="Show remote raw data value to verify that remote is work correctly.")

    args = parser.parse_args()

    # --- Setup Loggers ---
    log_dir = "logs"
    motor_logger = setup_logger('motor_logger', os.path.join(log_dir, 'motor.log'))
    if args.simulate:
        simulation_logger = setup_logger('simulation_logger', os.path.join(log_dir, 'simulation.log'))
        print(f"Logging simulation data to '{log_dir}/simulation.log'.")
    else:
        remote_logger = setup_logger('remote_logger', os.path.join(log_dir, 'remote.log'))
        lidar_logger = setup_logger('lidar_logger', os.path.join(log_dir, 'lidar.log'))
        gps_logger = setup_logger('gps_logger', os.path.join(log_dir, 'gps.log'))
        print(f"Logging sensor data to '{log_dir}/' directory.")

    # --- Initialize Components ---
    motor = Motor(logger=motor_logger)
    remote = None
    lidar = None
    gps = None

    if args.live:
        print("--- Running in LIVE mode ---")
        remote = RemoteControl(logger=remote_logger)
        lidar = Lidar(port="/dev/ttyUSB0")
        gps = GPS()
    elif args.simulate:
        from keyboard_control import KeyboardControl
        print("--- Running in SIMULATION mode ---")
        remote = KeyboardControl()
        lidar = Lidar(simulate=True)
    else: # File mode
        print("--- Running in SAMPLE-FILE mode ---")
        if not all([args.input_remote, args.input_lidar, args.input_gps]):
            parser.error("--input-remote, --input-lidar, and --input-gps are all required for file mode.")
        remote = RemoteControl(from_file=args.input_remote, logger=remote_logger)
        lidar = Lidar(from_file=args.input_lidar)
        gps = GPS(from_file=args.input_gps)

    # --- Main Control Loop ---
    try:
        if args.simulate:
            print("Starting simulation... Use W/S/A/D or arrow keys. Press Ctrl+C to exit.")
            motor.stop()

            while True:
                control_data = remote.read_data()
                lidar_data = lidar.read_data() # Read every loop

                if control_data:
                    # In simulation, we are always in manual mode
                    motor.move(control_data['throttle'], control_data['steering'])

                # Combine all info into one line
                control_log = "No remote data"
                if control_data:
                    control_log = (f"In: Thr={control_data['throttle']:.2f}, Steer={control_data['steering']:.2f} | "
                                   f"Out: L={motor.left_speed:.2f}, R={motor.right_speed:.2f}")

                lidar_log = "No lidar data"
                if lidar_data:
                    lidar_log = (
                        f"F:{lidar_data['front']:.1f} "
                        f"FR:{lidar_data['front_right']:.1f} "
                        f"R:{lidar_data['right']:.1f} "
                        f"BR:{lidar_data['back_right']:.1f} "
                        f"B:{lidar_data['back']:.2f} "
                        f"BL:{lidar_data['back_left']:.1f} "
                        f"L:{lidar_data['left']:.1f} "
                        f"FL:{lidar_data['front_left']:.1f}"
                    )

                full_log = f"{control_log} | LIDAR: {lidar_log}"
                simulation_logger.info(full_log)
                print(f"\r{full_log.ljust(120)}", end="") # ljust to clear previous line

                time.sleep(0.1) # Add a small sleep to prevent busy-waiting

        else:
            print("Starting control loop... Press Ctrl+C to exit.")
            running = True

            while running:
                # Read from all sensors
                remote_data = remote.read_data()
                lidar_data = lidar.read_data() if lidar else None
                gps_data = gps.read_data() if gps else None

                # Log sensor data if available
                if remote_data:
                    if args.show_remote_raw_data and 'raw_frame' in remote_data and remote_data['raw_frame']:
                        print(f"REMOTE RAW: {remote_data['raw_frame'].hex()}")

                    channels = remote_data.get('channels', [])
                    log_msg = (f"Mode: {remote_data['mode']}, "
                               f"Throttle: {remote_data['throttle']:.2f}, "
                               f"Steering: {remote_data['steering']:.2f}, "
                               f"Channels: {channels}")
                    remote_logger.info(log_msg)
                    # Display all channel values for debugging and real-time monitoring
                    ch_str = " | ".join([f"CH{i+1}: {v}" for i, v in enumerate(channels)])
                    print(f"REMOTE: Mode: {remote_data['mode']}, Thr: {remote_data['throttle']:.2f}, Steer: {remote_data['steering']:.2f}")
                    print(f"        {ch_str}")
                if lidar_data:
                    # Format the dictionary into a readable string for logging and printing
                    log_msg = (
                        f"Front: {lidar_data['front']:.2f}m, "
                        f"Front-Right: {lidar_data['front_right']:.2f}m, "
                        f"Right: {lidar_data['right']:.2f}m, "
                        f"Back-Right: {lidar_data['back_right']:.2f}m, "
                        f"Back: {lidar_data['back']:.2f}m, "
                        f"Back-Left: {lidar_data['back_left']:.2f}m, "
                        f"Left: {lidar_data['left']:.2f}m, "
                        f"Front-Left: {lidar_data['front_left']:.2f}m"
                    )
                    lidar_logger.info(log_msg)
                    # Also print to console for real-time monitoring
                    print(f"LIDAR: {log_msg}")
                if gps_data:
                    gps_logger.info(gps_data)
                    print(f"GPS: {gps_data}")

                # Process remote data
                if remote_data:
                    if remote_data['mode'] == 'manual':
                        motor.move(remote_data['throttle'], remote_data['steering'])
                        # Display the resulting motor speeds for real-time monitoring
                        print(f"MOTOR: Left={motor.left_speed:.2f}, Right={motor.right_speed:.2f}")
                    else: # Auto mode
                        # In auto mode, remote doesn't control motor.
                        # For now, we just stop the motor to be safe.
                        motor.stop()
                        print("AUTO MODE: Remote control disabled.")


                # Check for end of file in sample mode
                if not args.live and not remote_data and not lidar_data and not gps_data:
                    print("End of sample data files reached.")
                    running = False

                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt. Shutting down gracefully.")
    finally:
        motor.stop()
        motor.cleanup()
        if remote:
            remote.close()
        if lidar:
            lidar.close()
        if gps:
            gps.close()
        print("All resources have been released.")

if __name__ == "__main__":
    main()
