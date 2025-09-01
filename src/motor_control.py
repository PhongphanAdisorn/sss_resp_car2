from gpiozero import PWMOutputDevice
from time import sleep

# Based on the user's request for 50Hz frequency, the period is 20ms (1/50).
# I am assuming the user intended to use the standard hobby servo pulse widths:
# - Minimum: 1.0ms (for a value of -1.0)
# - Middle:  1.5ms (for a value of 0.0)
# - Maximum: 2.0ms (for a value of 1.0)
# The user's original values of 100us and 150us were too low for standard
# ESCs/servos, and the max value of 200ms was corrected to a more standard 2ms.

class MotorControl:
    """
    A wrapper class to control a motor or servo using PWM signals,
    mapping a value from -1.0 to 1.0 to a specific pulse width range.
    This is designed to work with the gpiozero.PWMOutputDevice as requested.
    """
    def __init__(self, pin, frequency=50):
        """
        Initializes the motor control.

        :param pin: The GPIO pin number to use.
        :param frequency: The PWM frequency in Hz. Defaults to 50Hz.
        """
        # Configuration is based on standard hobby servo/ESC specs.
        self.min_pulse_ms = 1.0
        self.max_pulse_ms = 2.0

        self.period_ms = 1000.0 / frequency
        self.pulse_range_ms = self.max_pulse_ms - self.min_pulse_ms
        self.mid_pulse_ms = self.min_pulse_ms + (self.pulse_range_ms / 2.0)

        self.device = PWMOutputDevice(pin=pin, frequency=frequency, initial_value=0)
        self.center() # Start at the neutral/center position.

    def set_value(self, value):
        """
        Sets the motor/servo position or speed.

        :param value: A value from -1.0 (e.g., full reverse, full left) to
                      1.0 (e.g., full forward, full right).
        """
        if not -1.0 <= value <= 1.0:
            raise ValueError("Value must be between -1.0 and 1.0")

        # Linearly map the input value [-1.0, 1.0] to the pulse width range [min, max]
        pulse_width_ms = self.mid_pulse_ms + (value * (self.pulse_range_ms / 2.0))

        # Calculate the duty cycle required for the PWMOutputDevice
        duty_cycle = pulse_width_ms / self.period_ms

        self.device.value = duty_cycle

    def center(self):
        """Sets the motor/servo to its neutral/center position (0.0)."""
        self.set_value(0)

    def close(self):
        """
        Stops the PWM signal and releases the GPIO resources.
        """
        self.device.close()


if __name__ == '__main__':
    # GPIO pins as per user request
    THROTTLE_PIN = 18
    STEERING_PIN = 19

    # Instantiate the controllers
    throttle = MotorControl(THROTTLE_PIN)
    steering = MotorControl(STEERING_PIN)

    print("Motor controllers initialized. GPIO18 for throttle, GPIO19 for steering.")
    print("Running a simple demonstration sequence...")

    try:
        # Example usage:
        # The script expects the ESC to be armed. This usually requires
        # sending a neutral signal (0) for a few seconds after power-on.
        print("Centering controls for 2 seconds to arm ESC...")
        throttle.center()
        steering.center()
        sleep(2)

        # --- Throttle demonstration ---
        print("Throttle half forward (0.5)")
        throttle.set_value(0.5) # Speed is capped at 1.0 by user request
        sleep(2)

        print("Throttle neutral (0.0)")
        throttle.center()
        sleep(2)

        # --- Steering demonstration ---
        print("Steering full left (-1.0)")
        steering.set_value(-1.0)
        sleep(2)

        print("Steering full right (1.0)")
        steering.set_value(1.0)
        sleep(2)

        print("Steering center (0.0)")
        steering.center()
        sleep(1)

        print("Demonstration complete.")

    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
    finally:
        # Clean up resources
        throttle.close()
        steering.close()
        print("GPIO resources released.")
