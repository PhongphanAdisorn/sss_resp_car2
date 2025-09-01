import logging
import time

# --- Configuration for Independent Throttle and Steering Motor Control ---
# User requested GPIO18 for throttle and GPIO19 for steering.
THROTTLE_GPIO = 18
STEERING_GPIO = 19

# --- PWM Configuration ---
# User requested a 50Hz frequency.
PWM_FREQUENCY = 50
# The period is 1 / frequency.
PWM_PERIOD = 1.0 / PWM_FREQUENCY  # This is 20ms for 50Hz.

# --- Pulse Width Configuration ---
# NOTE: The user specified pulse widths of 100us, 150us, and 200ms.
# A 200ms pulse width is not possible with a 50Hz (20ms) frequency.
# Standard servo pulse widths are typically 1.0ms to 2.0ms.
# We are assuming the user's request contained typos and we will use the
# standard 1.0ms (minimum), 1.5ms (neutral), and 2.0ms (maximum) pulse widths.
MIN_PULSE_WIDTH_S = 1.0 / 1000  # 1.0 ms
NEUTRAL_PULSE_WIDTH_S = 1.5 / 1000  # 1.5 ms
MAX_PULSE_WIDTH_S = 2.0 / 1000  # 2.0 ms

# Calculate duty cycles from pulse widths. Duty cycle = pulse_width / period.
MIN_DUTY_CYCLE = MIN_PULSE_WIDTH_S / PWM_PERIOD
NEUTRAL_DUTY_CYCLE = NEUTRAL_PULSE_WIDTH_S / PWM_PERIOD
MAX_DUTY_CYCLE = MAX_PULSE_WIDTH_S / PWM_PERIOD


# --- Dependency Check ---
try:
    # Per user request, use PWMOutputDevice directly.
    from gpiozero import PWMOutputDevice
    from gpiozero.pins.native import NativeFactory
    from gpiozero.pins.mock import MockFactory, MockPWMPin
    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False
    # Define dummy classes for simulation if gpiozero is not installed
    class PWMOutputDevice:
        def __init__(self, *args, **kwargs): self.value = 0
        def close(self): pass
    print("WARNING: gpiozero library not found. Motor control will be simulated.")


def map_range(x, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class Motor:
    """
    Controls a vehicle with independent throttle and steering.
    - Throttle: Controls forward/backward speed (e.g., an ESC).
    - Steering: Controls the direction of the front wheels (e.g., a servo).
    - Uses gpiozero.PWMOutputDevice to generate the required PWM signals.
    - Falls back to simulation if not on a Raspberry Pi or if gpiozero is missing.
    """
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)
        self.throttle_level = 0.0
        self.steering_level = 0.0
        self.is_pi = False

        pin_factory = None
        if GPIOZERO_AVAILABLE:
            try:
                pin_factory = NativeFactory()
                self.is_pi = True
                print("Running in LIVE motor mode on Raspberry Pi.")
            except Exception:
                pin_factory = MockFactory(pin_class=MockPWMPin)
                print("Running in SIMULATED motor mode (gpiozero is installed).")
        else:
            print("Running in SIMULATED motor mode (gpiozero not installed).")

        # Initialize PWMOutputDevice for throttle and steering
        self.throttle_motor = PWMOutputDevice(
            pin=THROTTLE_GPIO,
            frequency=PWM_FREQUENCY,
            pin_factory=pin_factory
        )
        self.steering_servo = PWMOutputDevice(
            pin=STEERING_GPIO,
            frequency=PWM_FREQUENCY,
            pin_factory=pin_factory
        )
        self.stop()
        self.logger.info("Motor controller initialized for independent throttle/steering.")

    def move(self, throttle, steering):
        """
        Sets the throttle and steering based on input values.
        :param throttle: Float from -1.0 (full reverse) to 1.0 (full forward).
        :param steering: Float from -1.0 (full left) to 1.0 (full right).
        """
        self.throttle_level = max(-1.0, min(1.0, throttle))
        self.steering_level = max(-1.0, min(1.0, steering))

        # Map the [-1, 1] input to the appropriate duty cycle range.
        # For many ESCs and servos, this range corresponds to 1ms-2ms pulse width.
        self.throttle_motor.value = map_range(
            self.throttle_level, -1, 1, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE
        )
        self.steering_servo.value = map_range(
            self.steering_level, -1, 1, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE
        )

        log_msg = (f"Move(thr={self.throttle_level:.2f}, steer={self.steering_level:.2f}) -> "
                   f"Throttle Duty: {self.throttle_motor.value:.4f}, "
                   f"Steering Duty: {self.steering_servo.value:.4f}")
        self.logger.info(log_msg)
        if not self.is_pi:
            print(f"SIM_INFO: {log_msg}")

    def stop(self):
        """Stops the motor and centers the steering."""
        self.throttle_motor.value = NEUTRAL_DUTY_CYCLE
        self.steering_servo.value = NEUTRAL_DUTY_CYCLE
        self.throttle_level = 0.0
        self.steering_level = 0.0
        self.logger.info("Motor stopped, steering centered.")
        print("ACTION: Motor stopped, steering centered.")

    def cleanup(self):
        """Releases GPIO resources."""
        self.stop()
        self.throttle_motor.close()
        self.steering_servo.close()
        self.logger.info("Motor resources cleaned up.")
        print("Motor resources cleaned up.")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    test_logger = logging.getLogger('test_motor')

    motor = Motor(logger=test_logger)
    try:
        print("\n--- Testing Independent Throttle & Steering Control ---")

        print("\n1. Centering controls")
        motor.stop()
        time.sleep(2)

        print("\n2. Full throttle forward")
        motor.move(throttle=1.0, steering=0.0)
        time.sleep(2)

        print("\n3. Half throttle reverse")
        motor.move(throttle=-0.5, steering=0.0)
        time.sleep(2)

        print("\n4. Steering full right")
        motor.move(throttle=0.0, steering=1.0)
        time.sleep(2)

        print("\n5. Steering full left")
        motor.move(throttle=0.0, steering=-1.0)
        time.sleep(2)

        print("\n6. Half forward, half-right turn")
        motor.move(throttle=0.5, steering=0.5)
        time.sleep(2)

        print("\n7. Stopping")
        motor.stop()
        time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        motor.cleanup()
    print("\n--- Motor Test Complete ---")
