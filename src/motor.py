import logging
import time

# --- Configuration for Dual PWM Motor Control ---
LEFT_MOTOR_GPIO = 18
RIGHT_MOTOR_GPIO = 19

# Standard PWM Configuration for Servos/ESCs
MIN_PULSE_WIDTH = 1.0 / 1000  # 1.0 ms (Full Reverse)
MID_PULSE_WIDTH = 1.5 / 1000  # 1.5 ms (Neutral)
MAX_PULSE_WIDTH = 2.0 / 1000  # 2.0 ms (Full Forward)
FRAME_WIDTH = 20 / 1000       # 20 ms (50 Hz frame rate)

# --- Dependency Check ---
try:
    from gpiozero import Servo
    from gpiozero.pins.native import NativeFactory
    from gpiozero.pins.mock import MockFactory, MockPWMPin
    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False
    # Define dummy classes for simulation if gpiozero is not installed
    class Servo:
        def __init__(self, *args, **kwargs): self.value = 0
        def close(self): pass
    print("WARNING: gpiozero library not found. Motor control will be simulated.")


class Motor:
    """
    Controls a differential drive robot using two separate PWM signals, one for
    each motor (left and right), via GPIO 18 and 19.
    - Translates high-level throttle/steering commands into left/right motor values.
    - Uses gpiozero.Servo to generate the required PWM signals.
    - Falls back to simulation if not on a Raspberry Pi or if gpiozero is missing.
    """
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)
        self.left_speed = 0.0
        self.right_speed = 0.0
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

        # Initialize Servo objects for left and right motors
        self.left_motor = Servo(
            pin=LEFT_MOTOR_GPIO,
            initial_value=0,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            frame_width=FRAME_WIDTH,
            pin_factory=pin_factory
        )
        self.right_motor = Servo(
            pin=RIGHT_MOTOR_GPIO,
            initial_value=0,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            frame_width=FRAME_WIDTH,
            pin_factory=pin_factory
        )
        self.stop()
        self.logger.info("Motor controller initialized for Dual PWM Control.")

    def move(self, throttle, steering):
        """
        Calculates left/right motor speeds from throttle and steering inputs.
        :param throttle: Float from -1.0 (reverse) to 1.0 (forward).
        :param steering: Float from -1.0 (left) to 1.0 (right).
        """
        throttle = max(-1.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))

        # Mixing algorithm
        left = throttle + steering
        right = throttle - steering

        # Clamp values to the [-1, 1] range
        self.left_speed = max(-1.0, min(1.0, left))
        self.right_speed = max(-1.0, min(1.0, right))

        # Set motor values. The Servo class maps -1 to 1 to the pulse width.
        # For a standard setup, right motor needs to be inverted for steering.
        self.left_motor.value = self.left_speed
        self.right_motor.value = self.right_speed

        log_msg = (f"Move(thr={throttle:.2f}, steer={steering:.2f}) -> "
                   f"Left: {self.left_speed:.2f}, Right: {self.right_speed:.2f}")
        self.logger.info(log_msg)
        if not self.is_pi:
            print(f"SIM_INFO: {log_msg}")

    def stop(self):
        """Stops both motors."""
        self.left_motor.value = 0
        self.right_motor.value = 0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.logger.info("Motors stopped.")
        print("ACTION: Motors STOPPED.")

    def cleanup(self):
        """Releases GPIO resources."""
        self.stop()
        self.left_motor.close()
        self.right_motor.close()
        self.logger.info("Motor resources cleaned up.")
        print("Motor resources cleaned up.")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    test_logger = logging.getLogger('test_motor')

    motor = Motor(logger=test_logger)
    try:
        print("\n--- Testing Dual PWM Motor Control ---")

        print("\n1. Moving Forward (Throttle=0.6)")
        motor.move(throttle=0.6, steering=0)
        time.sleep(2)

        print("\n2. Moving Backward (Throttle=-0.6)")
        motor.move(throttle=-0.6, steering=0)
        time.sleep(2)

        print("\n3. Spinning Right (Steering=0.5)")
        motor.move(throttle=0, steering=0.5)
        time.sleep(2)

        print("\n4. Spinning Left (Steering=-0.5)")
        motor.move(throttle=0, steering=-0.5)
        time.sleep(2)

        print("\n5. Gentle Forward-Left (Throttle=0.5, Steering=-0.3)")
        motor.move(throttle=0.5, steering=-0.3)
        time.sleep(2)

        print("\n6. Stopping")
        motor.stop()
        time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        motor.cleanup()
    print("\n--- Motor Test Complete ---")
