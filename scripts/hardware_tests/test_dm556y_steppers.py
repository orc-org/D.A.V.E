#!/usr/bin/env python3
"""
Hardware Test Script for DM556Y Stepper Driver on Jetson Orin Nano
Wired Pins:
  PUL+ -> Jetson Pin 15 (GPIO12 / PWM)
  DIR+ -> Jetson Pin 7  (GPIO9)
  PUL-, DIR- -> Jetson GND (Pin 6/9/14)
"""

import time
import sys

try:
    import Jetson.GPIO as GPIO
except ImportError:
    print("Error: Jetson.GPIO library is not installed.")
    sys.exit(1)

# Physical (BOARD) pin numbers. These are what the script drives. The docstring above and the
# arm/gripper nodes disagree with them (arm uses STEP 33 / DIR 18, gripper STEP 32 / DIR 13);
# check the wiring and edit here before running. See docs/known-issues.md.
PUL_PIN = 15  # Hardware PWM capable pin
DIR_PIN = 13  # Direction (this is the gripper DIR pin in the arm node)

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PUL_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)

def test_pulse_frequency(target_freq_hz=20000, duration_sec=3.0, forward=True):
    """
    Ramps frequency up to target_freq_hz using Jetson Hardware PWM
    so the stepper motor accelerates smoothly without stalling.
    """
    print(f"\n-> Setting Direction: {'FORWARD (HIGH)' if forward else 'REVERSE (LOW)'}")
    GPIO.output(DIR_PIN, GPIO.HIGH if forward else GPIO.LOW)
    time.sleep(0.01)

    print(f"-> Starting PWM Frequency Ramping up to {target_freq_hz/1000:.1f} kHz...")
    
    pwm = GPIO.PWM(PUL_PIN, 1000)
    pwm.start(50)  # 50% duty cycle square wave
    
    # Smooth acceleration ramp from 1kHz to target_freq_hz over 0.5s
    steps_ramp = 50
    start_freq = 1000
    for i in range(steps_ramp + 1):
        current_freq = int(start_freq + (target_freq_hz - start_freq) * (i / steps_ramp))
        pwm.ChangeFrequency(current_freq)
        time.sleep(0.01)

    print(f"-> Holding at top speed ({target_freq_hz/1000:.1f} kHz) for {duration_sec}s...")
    time.sleep(duration_sec)

    # Deceleration ramp down to 1kHz
    print("-> Decelerating...")
    for i in range(steps_ramp, -1, -1):
        current_freq = int(start_freq + (target_freq_hz - start_freq) * (i / steps_ramp))
        pwm.ChangeFrequency(current_freq)
        time.sleep(0.01)

    pwm.stop()
    print("-> Stopped.")

def main():
    print("=" * 65)
    print(" DM556Y High-Speed PWM Stepper Motor Test (Jetson Orin Nano)")
    print("=" * 65)
    print(f" PUL+ -> Jetson Pin {PUL_PIN} (Hardware PWM)")
    print(f" DIR+ -> Jetson Pin {DIR_PIN} (Direction)")
    print(" PUL-, DIR- -> Jetson GND")
    print("-" * 65)

    target_freq = 20000  # Default 20 kHz (12.5 RPS / 750 RPM at 1600 microstep); was 200000, a typo
    if len(sys.argv) > 1:
        try:
            target_freq = int(sys.argv[1])
        except ValueError:
            pass

    # A mistyped argument (the old default was already 10x too high) would stall a loaded stepper
    # or run an arm into its stops, so refuse implausible values unless explicitly forced.
    MAX_SAFE_HZ = 25000
    if target_freq <= 0 or (target_freq > MAX_SAFE_HZ and '--force' not in sys.argv):
        print(f"Refusing {target_freq} Hz: use 1..{MAX_SAFE_HZ} Hz, or add --force to override.")
        sys.exit(2)

    print(f"Target Max Frequency: {target_freq} Hz ({target_freq/1000:.1f} kHz)")

    try:
        setup()

        # Run Forward Test
        test_pulse_frequency(target_freq_hz=target_freq, duration_sec=3.0, forward=True)
        time.sleep(1.0)

        # Run Reverse Test
        test_pulse_frequency(target_freq_hz=target_freq, duration_sec=3.0, forward=False)

        print("\n✅ Spin Test Complete!")

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass

if __name__ == '__main__':
    main()
