import keyboard
import time
import threading

class KeyboardControl:
    """
    Handles reading keyboard inputs for controlling the robot in simulation mode.
    Maintains state for speed and steering and returns a data dictionary.
    """
    def __init__(self):
        self.throttle = 0.0
        self.steering = 0.0
        self.lock = threading.Lock()
        self.updated = False

        # Note: This may require root privileges on Linux
        keyboard.on_press(self._on_press, suppress=False)
        print("KeyboardControl initialized. Use arrow keys to control. W/S for speed, A/D for steering.")

    def _on_press(self, event):
        """Callback function for key presses."""
        with self.lock:
            if event.name == 'w' or event.name == 'up':
                self.throttle = min(1.0, self.throttle + 0.1)
            elif event.name == 's' or event.name == 'down':
                self.throttle = max(-1.0, self.throttle - 0.1)
            elif event.name == 'a' or event.name == 'left':
                self.steering = max(-1.0, self.steering - 0.1)
            elif event.name == 'd' or event.name == 'right':
                self.steering = min(1.0, self.steering + 0.1)
            elif event.name == 'space': # Stop
                self.throttle = 0.0
                self.steering = 0.0
            self.updated = True

    def read_data(self):
        """
        Returns the current control data if there has been an update.
        """
        with self.lock:
            if not self.updated:
                return None

            data = {
                'mode': 'manual',
                'throttle': self.throttle,
                'steering': self.steering
            }
            self.updated = False # Reset after reading
            return data

    def close(self):
        """Stops the keyboard listener."""
        # Unhook all keyboard events
        keyboard.unhook_all()
        print("KeyboardControl listener stopped.")

if __name__ == '__main__':
    # Example usage and test
    print("--- KeyboardControl Test ---")
    print("Press arrow keys to see the commands. Press Ctrl+C to exit after a few seconds.")
    print("NOTE: This test might require running the script with 'sudo'.")

    kb = KeyboardControl()

    try:
        # Keep the main thread alive to allow the background listener to run
        for _ in range(20): # Run for 10 seconds
            cmd = kb.read_command()
            if cmd:
                print(f"Received command: {cmd}")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt caught, exiting.")
    finally:
        kb.close()

    print("--- KeyboardControl Test Complete ---")
