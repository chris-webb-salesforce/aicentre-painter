import time
import numpy as np
from pymycobot import MyCobot320
from config import *
from workspace_manager import SurfaceCalibrator


class RobotController:
    def __init__(self):
        print("Initializing Robot Controller...")
        try:
            self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
            time.sleep(2)
            self.mc.set_fresh_mode(1)
            print("Fresh mode enabled")

        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            raise

        self.surface = SurfaceCalibrator()

        if self.surface.calibration_data:
            print("Using calibrated orientation from file.")
            self.drawing_orientation = self.surface.calibration_data['orientation']
        else:
            print("No calibration found. Using default orientation from config.")
            self.drawing_orientation = DRAWING_ORIENTATION

        self.pen_is_down = False
        self.current_position = None
        self.last_command_time = 0

    def validate_z_coordinate(self, z):
        if z < MIN_SAFE_Z:
            print(f"WARNING: Z coordinate {z:.2f} below minimum {MIN_SAFE_Z:.2f}. Clamping.")
            return MIN_SAFE_Z + SAFETY_MARGIN
        elif z > MAX_SAFE_Z:
            print(f"WARNING: Z coordinate {z:.2f} above maximum {MAX_SAFE_Z:.2f}. Clamping.")
            return MAX_SAFE_Z - SAFETY_MARGIN
        return z

    def wait_for_completion(self, target_position=None, timeout=None):
        if timeout is None:
            timeout = MOVEMENT_TIMEOUT

        start_time = time.time()
        stable_readings = 0
        required_stable_readings = 2
        last_position = None

        while time.time() - start_time < timeout:
            try:
                current_pos = self.mc.get_coords()

                if current_pos and len(current_pos) >= 3:
                    if last_position:
                        movement = np.sqrt(sum((current_pos[i] - last_position[i])**2 for i in range(3)))

                        if movement < 0.5:
                            stable_readings += 1
                            if stable_readings >= required_stable_readings:
                                time.sleep(MOVEMENT_SETTLING_TIME)
                                return True
                        else:
                            stable_readings = 0

                    last_position = current_pos[:]

                time.sleep(POSITION_CHECK_INTERVAL)

            except Exception as e:
                print(f"Motion detection error: {e}")
                time.sleep(0.1)
                break

        print(f"Movement timeout after {timeout}s")
        return False

    def move_to(self, x, y, z, speed=None, orientation=None):
        """Move to absolute coordinates."""
        if speed is None:
            speed = TRAVEL_SPEED

        if orientation is None:
            orientation = self.drawing_orientation

        try:
            current_time = time.time()
            time_since_last = current_time - self.last_command_time
            if time_since_last < MIN_COMMAND_INTERVAL:
                time.sleep(MIN_COMMAND_INTERVAL - time_since_last)

            z = self.validate_z_coordinate(z)
            coords = [x, y, z] + orientation

            self.mc.send_coords(coords, speed, 0)
            self.last_command_time = time.time()

            if USE_MOVEMENT_SYNC:
                success = self.wait_for_completion([x, y, z], MOVEMENT_TIMEOUT)
                if not success:
                    print(f"Move to ({x:.1f}, {y:.1f}, {z:.1f}) may not have completed")
                return success
            else:
                time.sleep(0.5)
                return True

        except Exception as e:
            print(f"Move failed: {e}")
            return False

    def move_relative_to_surface(self, x, y, offset_z, speed=None):
        """Move to X,Y at Z calculated from calibrated surface + offset."""
        surface_z = self.surface.get_z_at(x, y)
        target_z = surface_z + offset_z
        return self.move_to(x, y, target_z, speed, self.drawing_orientation)

    def pen_down(self):
        if self.pen_is_down:
            return True

        print("Lowering pen...")

        try:
            if not self.current_position:
                current = self.mc.get_coords()
                if not current:
                    return False
                self.current_position = [current[0], current[1]]

            x, y = self.current_position

            success = self.move_relative_to_surface(x, y, DRAWING_HEIGHT_OFFSET, APPROACH_SPEED)

            if not success:
                print("Pen lowering failed")
                return False

            self.pen_is_down = True
            print(f"Pen down at ({x:.1f}, {y:.1f})")
            return True

        except Exception as e:
            print(f"Pen down failed: {e}")
            return False

    def pen_up(self):
        if not self.pen_is_down:
            return True

        try:
            if not self.current_position:
                current = self.mc.get_coords()
                if not current:
                    return False
                self.current_position = [current[0], current[1]]

            x, y = self.current_position

            success = self.move_relative_to_surface(x, y, LIFT_HEIGHT_OFFSET, LIFT_SPEED)

            if not success:
                print("Pen lifting failed")
                return False

            self.pen_is_down = False
            print(f"Pen up from ({x:.1f}, {y:.1f})")
            return True

        except Exception as e:
            print(f"Pen up failed: {e}")
            return False

    def draw_to(self, x, y):
        """Draw line to X,Y maintaining surface contact."""
        if not self.pen_is_down:
            print("Pen not down, call pen_down() first")
            return False

        success = self.move_relative_to_surface(x, y, DRAWING_HEIGHT_OFFSET, DRAWING_SPEED)

        if success:
            self.current_position = [x, y]

        return success

    def draw_full_sketch(self, contours):
        """Draw all contours."""
        print(f"Starting sketch: {len(contours)} contours.")

        for i, contour in enumerate(contours):
            if len(contour) < 2:
                continue

            start_x, start_y = contour[0][0], contour[0][1]
            self.move_relative_to_surface(start_x, start_y, LIFT_HEIGHT_OFFSET, TRAVEL_SPEED)
            self.pen_down()

            for point in contour[1:]:
                self.draw_to(point[0], point[1])

            self.pen_up()

            if i % 5 == 0:
                print(f"Progress: {i+1}/{len(contours)} contours")

        print("Sketch complete. Moving Home.")
        self.go_to_home_position()

    def go_to_home_position(self):
        print("Moving to home position...")
        self.pen_up()
        time.sleep(0.5)
        return self.move_to(ORIGIN_X, ORIGIN_Y, SAFE_TRAVEL_HEIGHT, TRAVEL_SPEED, DRAWING_ORIENTATION)

    def go_to_safe_position(self):
        print("Moving to safe position...")
        self.pen_up()
        time.sleep(0.5)

        try:
            self.mc.send_angles([0, 0, 0, 0, 90, DESIRED_J6_ANGLE], 30)
            time.sleep(3)
            return True
        except Exception as e:
            print(f"Failed to go to safe position: {e}")
            return False

    def get_current_position(self):
        try:
            angles = self.mc.get_angles()
            coords = self.mc.get_coords()
            return {"angles": angles, "coords": coords}
        except Exception as e:
            print(f"Failed to get position: {e}")
            return None