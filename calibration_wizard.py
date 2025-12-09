import sys
import tty
import termios
import json
import time
from pymycobot import MyCobot320
from config import *

MOVE_SPEED = 30
STEP_LARGE = 10.0
STEP_SMALL = 2.0
STEP_MICRO = 0.5


class CalibrationWizard:
    def __init__(self):
        print("Connecting to robot...")
        self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
        time.sleep(1)
        self.mc.power_on()
        self.step_size = STEP_LARGE

        self.points = {
            "p1": None,
            "p2": None,
            "p3": None
        }

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def print_status(self):
        coords = self.mc.get_coords()
        if not coords:
            return

        print("\033[H\033[J")
        print("=" * 50)
        print("    SURFACE CALIBRATION WIZARD (Angled Surface)")
        print("=" * 50)
        print(f"\nPosition:  X={coords[0]:7.1f}  Y={coords[1]:7.1f}  Z={coords[2]:7.1f}")
        print(f"Rotation: Rx={coords[3]:7.1f} Ry={coords[4]:7.1f} Rz={coords[5]:7.1f}")
        print(f"\nStep Size: {self.step_size}mm  [1]=10mm [2]=2mm [3]=0.5mm")
        print("-" * 50)
        print("MOVEMENT:")
        print("  W/S = X axis    A/D = Y axis    Q/E = Z axis")
        print("ROTATION (for angled surface):")
        print("  R/F = Rx (pitch)   T/G = Ry (roll)   Y/H = Rz (yaw)")
        print("-" * 50)
        print("CALIBRATION POINTS (touch pen to paper corner):")

        for key, name in [("p1", "TOP-LEFT"), ("p2", "TOP-RIGHT"), ("p3", "BOTTOM-LEFT")]:
            if self.points[key]:
                p = self.points[key]
                print(f"  [{key[-1]}] {name:12} X={p[0]:6.1f} Y={p[1]:6.1f} Z={p[2]:6.1f}")
            else:
                print(f"  [{key[-1]}] {name:12} -- not saved --")

        print("-" * 50)
        print("  [V] = Verify calibration (test center point)")
        print("  [X] = Save & Exit")
        print("  Ctrl+C = Quit without saving")

    def verify_calibration(self):
        if not all(self.points.values()):
            print("\nSave all 3 points first!")
            time.sleep(1)
            return

        p1 = self.points['p1']
        p2 = self.points['p2']
        p3 = self.points['p3']

        center_x = (p1[0] + p2[0] + p3[0]) / 3
        center_y = (p1[1] + p2[1] + p3[1]) / 3

        from workspace_manager import SurfaceCalibrator
        temp_cal = SurfaceCalibrator.__new__(SurfaceCalibrator)
        temp_cal.plane_coeffs = None
        temp_cal.calculate_plane(p1, p2, p3)

        center_z = temp_cal.get_z_at(center_x, center_y)

        print(f"\nMoving to center: X={center_x:.1f} Y={center_y:.1f}")
        print(f"Calculated Z for surface: {center_z:.1f}")
        print("Press any key to move there (pen should touch paper)...")
        self.get_key()

        orientation = [p1[3], p1[4], p1[5]]
        self.mc.send_coords([center_x, center_y, center_z] + orientation, MOVE_SPEED, 0)
        time.sleep(2)

        print("\nDoes the pen touch the paper correctly?")
        print("  [Y] = Yes, calibration looks good")
        print("  [N] = No, redo calibration")

        key = self.get_key().lower()
        if key == 'y':
            print("\nCalibration verified!")
        else:
            print("\nTry recalibrating the 3 points.")

        time.sleep(1)

    def run(self):
        print("Starting Wizard...")
        current_coords = self.mc.get_coords()
        if not current_coords:
            print("Error: Could not read robot coordinates.")
            return

        while True:
            self.print_status()
            key = self.get_key().lower()

            coords = self.mc.get_coords()
            if not coords:
                continue

            x, y, z, rx, ry, rz = coords

            if key == 'w':
                x += self.step_size
            elif key == 's':
                x -= self.step_size
            elif key == 'a':
                y += self.step_size
            elif key == 'd':
                y -= self.step_size
            elif key == 'q':
                z -= self.step_size
            elif key == 'e':
                z += self.step_size

            elif key == 'r':
                rx += 2.0
            elif key == 'f':
                rx -= 2.0
            elif key == 't':
                ry += 2.0
            elif key == 'g':
                ry -= 2.0
            elif key == 'y':
                rz += 2.0
            elif key == 'h':
                rz -= 2.0

            elif key == '1':
                self.step_size = STEP_LARGE
                continue
            elif key == '2':
                self.step_size = STEP_SMALL
                continue
            elif key == '3':
                self.step_size = STEP_MICRO
                continue

            elif key == '4':
                self.points['p1'] = list(coords)
                print("\nSaved TOP-LEFT!")
                time.sleep(0.5)
                continue
            elif key == '5':
                self.points['p2'] = list(coords)
                print("\nSaved TOP-RIGHT!")
                time.sleep(0.5)
                continue
            elif key == '6':
                self.points['p3'] = list(coords)
                print("\nSaved BOTTOM-LEFT!")
                time.sleep(0.5)
                continue

            elif key == 'v':
                self.verify_calibration()
                continue

            elif key == 'x':
                if all(self.points.values()):
                    self.save_to_file()
                    break
                else:
                    print("\nError: Save all 3 points first.")
                    time.sleep(1)
                    continue

            elif ord(key) == 3:
                print("\nQuitting without saving.")
                break

            else:
                continue

            self.mc.send_coords([x, y, z, rx, ry, rz], MOVE_SPEED, 0)
            time.sleep(0.1)

    def save_to_file(self):
        p1 = self.points['p1']
        orientation = [p1[3], p1[4], p1[5]]

        data = {
            "p1": self.points['p1'],
            "p2": self.points['p2'],
            "p3": self.points['p3'],
            "orientation": orientation
        }

        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(data, f, indent=4)
        print("\n\nCALIBRATION SAVED! Exiting...")


if __name__ == "__main__":
    wizard = CalibrationWizard()
    wizard.run()
