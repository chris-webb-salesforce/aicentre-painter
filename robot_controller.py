"""
Core robot control module for MyCobot 320.
SIMPLIFIED - Reliable movement with proper synchronization.
"""

import time
import numpy as np
from pymycobot import MyCobot320
from config import *


class RobotController:
    """Simple, reliable robot control."""

    def __init__(self):
        """Initialize robot controller."""
        print("Initializing Robot Controller...")
        try:
            self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
            time.sleep(2)  # Give robot time to initialize

            # Enable fresh mode for real-time commands
            self.mc.set_fresh_mode(1)
            print("✅ Fresh mode enabled")

        except Exception as e:
            print(f"❌ Failed to connect to robot: {e}")
            raise

        # Track state
        self.pen_is_down = False
        self.current_position = None
        self.last_command_time = 0

        print("✅ Robot Controller initialized")

    def validate_z_coordinate(self, z):
        """Validate Z coordinate for safety and clamp to safe range."""
        if z < MIN_SAFE_Z:
            print(f"⚠️  WARNING: Z coordinate {z:.2f} below minimum {MIN_SAFE_Z:.2f}. Clamping.")
            return MIN_SAFE_Z + SAFETY_MARGIN
        elif z > MAX_SAFE_Z:
            print(f"⚠️  WARNING: Z coordinate {z:.2f} above maximum {MAX_SAFE_Z:.2f}. Clamping.")
            return MAX_SAFE_Z - SAFETY_MARGIN
        return z

    def wait_for_completion(self, target_position=None, timeout=None):
        """
        Wait until robot has completely stopped moving.
        This is THE critical function for reliability.
        """
        if timeout is None:
            timeout = MOVEMENT_TIMEOUT

        start_time = time.time()
        stable_readings = 0
        required_stable_readings = 3  # Must be stable for 3 consecutive readings
        last_position = None

        while time.time() - start_time < timeout:
            try:
                current_pos = self.mc.get_coords()

                if current_pos and len(current_pos) >= 3:
                    if last_position:
                        # Calculate movement between readings
                        movement = np.sqrt(sum((current_pos[i] - last_position[i])**2 for i in range(3)))

                        if movement < 0.2:  # Very small movement threshold
                            stable_readings += 1
                            if stable_readings >= required_stable_readings:
                                time.sleep(MOVEMENT_SETTLING_TIME)  # Final settling
                                return True
                        else:
                            stable_readings = 0  # Reset if still moving

                    last_position = current_pos[:]

                time.sleep(POSITION_CHECK_INTERVAL)

            except Exception as e:
                print(f"⚠️  Motion detection error: {e}")
                time.sleep(0.1)
                break

        print(f"⚠️  Movement timeout after {timeout}s")
        return False

    def move_to(self, x, y, z=None, speed=None):
        """
        Move to position and WAIT for completion.
        This is the core movement function.
        """
        if z is None:
            z = SAFE_TRAVEL_HEIGHT
        if speed is None:
            speed = TRAVEL_SPEED

        try:
            # Enforce minimum command interval
            current_time = time.time()
            time_since_last = current_time - self.last_command_time
            if time_since_last < MIN_COMMAND_INTERVAL:
                time.sleep(MIN_COMMAND_INTERVAL - time_since_last)

            # Validate Z coordinate
            z = self.validate_z_coordinate(z)

            # Build coordinate command
            coords = [x, y, z] + DRAWING_ORIENTATION

            # Send command
            self.mc.send_coords(coords, speed, 0)  # Mode 0 = joint interpolation
            self.last_command_time = time.time()

            # WAIT for completion
            if USE_MOVEMENT_SYNC:
                success = self.wait_for_completion([x, y, z], MOVEMENT_TIMEOUT)
                if not success:
                    print(f"⚠️  Move to ({x:.1f}, {y:.1f}, {z:.1f}) may not have completed")
                return success
            else:
                # Fallback timing if sync disabled
                time.sleep(0.5)
                return True

        except Exception as e:
            print(f"❌ Move failed: {e}")
            return False

    def pen_down(self):
        """Lower pen to drawing height."""
        if self.pen_is_down:
            return True

        print("Lowering pen...")

        try:
            # Get current position
            current = self.mc.get_coords()
            if not current or len(current) < 3:
                print("⚠️  Could not get current position")
                return False

            x, y = current[0], current[1]

            # Gradual approach to drawing surface
            approach_steps = 3
            start_z = PEN_RETRACT_Z
            end_z = PEN_DRAWING_Z

            for i in range(1, approach_steps + 1):
                z_position = start_z - ((start_z - end_z) * (i / approach_steps))
                success = self.move_to(x, y, z_position, APPROACH_SPEED)
                if not success:
                    print(f"⚠️  Pen lowering step {i} failed")
                    return False

            self.pen_is_down = True
            self.current_position = [x, y]
            print(f"✓ Pen down at ({x:.1f}, {y:.1f})")
            return True

        except Exception as e:
            print(f"❌ Pen down failed: {e}")
            return False

    def pen_up(self):
        """Raise pen to safe height."""
        if not self.pen_is_down:
            return True

        print("Raising pen...")

        try:
            # Get current position
            current = self.mc.get_coords()
            if not current or len(current) < 3:
                print("⚠️  Could not get current position")
                return False

            x, y = current[0], current[1]

            # Gradual lift
            lift_steps = 2
            start_z = PEN_DRAWING_Z
            end_z = PEN_RETRACT_Z

            for i in range(1, lift_steps + 1):
                z_position = start_z + ((end_z - start_z) * (i / lift_steps))
                z_position = self.validate_z_coordinate(z_position)
                success = self.move_to(x, y, z_position, LIFT_SPEED)
                if not success:
                    print(f"⚠️  Pen lifting step {i} failed")
                    return False

            self.pen_is_down = False
            print(f"✓ Pen up from ({x:.1f}, {y:.1f})")
            return True

        except Exception as e:
            print(f"❌ Pen up failed: {e}")
            return False

    def draw_to(self, x, y):
        """
        Draw line to point (pen must be down).
        Simple, reliable drawing.
        """
        if not self.pen_is_down:
            print("⚠️  Pen not down, call pen_down() first")
            return False

        # Draw at fixed drawing height
        success = self.move_to(x, y, PEN_DRAWING_Z, DRAWING_SPEED)

        if success:
            self.current_position = [x, y]

        return success

    def go_to_home_position(self):
        """Move to drawing start position."""
        print("Moving to home position...")
        self.pen_up()
        time.sleep(0.5)
        return self.move_to(ORIGIN_X, ORIGIN_Y, SAFE_TRAVEL_HEIGHT, TRAVEL_SPEED)

    def go_to_safe_position(self):
        """Move to neutral safe position."""
        print("Moving to safe position...")
        self.pen_up()
        time.sleep(0.5)

        try:
            self.mc.send_angles([0, 0, 0, 0, 90, DESIRED_J6_ANGLE], 30)
            time.sleep(3)
            return True
        except Exception as e:
            print(f"❌ Failed to go to safe position: {e}")
            return False

    def get_current_position(self):
        """Get current robot position."""
        try:
            angles = self.mc.get_angles()
            coords = self.mc.get_coords()
            return {"angles": angles, "coords": coords}
        except Exception as e:
            print(f"❌ Failed to get position: {e}")
            return None
