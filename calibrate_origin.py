import time
from pymycobot import MyCobot320
import sys

# Robot configuration
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

class OriginCalibrator:
    def __init__(self):
        print("Initializing Origin Calibrator...")
        try:
            self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
            time.sleep(2)
        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            sys.exit(1)
        
        # Standard drawing orientation for vertical surface
        self.ORIENTATION = [180, -90, -90]
        
        # Starting position (adjust these as a starting point)
        self.current_x = 200.0
        self.current_y = -85.0
        self.current_z = 110.0
        
    def go_to_home(self):
        """Move to safe home position."""
        print("Moving to home position...")
        self.mc.send_angles([0, 0, 0, 0, 90, 0], 40)
        time.sleep(3)
    
    def move_to_current_position(self):
        """Move to the current calibration position."""
        coords = [self.current_x, self.current_y, self.current_z] + self.ORIENTATION
        self.mc.send_coords(coords, 20, 0)
        time.sleep(0.5)
    
    def calibrate(self):
        """Interactive calibration routine."""
        print("\n=== ORIGIN POINT CALIBRATION ===")
        print("\nThis will help you find the top-left corner of your drawing area.")
        print("Place your paper/canvas on the vertical surface.")
        print("\nIMPORTANT: The pen should barely touch the paper at the origin.")
        
        self.go_to_home()
        
        print("\n--- STEP 1: ROUGH POSITIONING ---")
        print("First, let's get close to your drawing surface.")
        print("\nCurrent position:")
        print(f"  X (left/right): {self.current_x:.1f}mm")
        print(f"  Y (forward/back): {self.current_y:.1f}mm") 
        print(f"  Z (up/down): {self.current_z:.1f}mm")
        
        input("\nPress ENTER to move to initial position...")
        self.move_to_current_position()
        
        print("\n--- STEP 2: COARSE ADJUSTMENT ---")
        print("Use these commands to position the pen near the top-left corner:")
        print("  'w' = Move UP (Z+)")
        print("  's' = Move DOWN (Z-)")
        print("  'a' = Move LEFT (X-)")
        print("  'd' = Move RIGHT (X+)")
        print("  'q' = Move FORWARD toward wall (Y+)")
        print("  'e' = Move BACKWARD from wall (Y-)")
        print("  'n' = Continue to FINE adjustment")
        print("\nMove in 10mm increments:")
        
        step_size = 10.0
        
        while True:
            command = input("Command: ").lower().strip()
            
            if command == 'w':
                self.current_z += step_size
                print(f"Moving up... Z = {self.current_z:.1f}")
            elif command == 's':
                self.current_z -= step_size
                print(f"Moving down... Z = {self.current_z:.1f}")
            elif command == 'a':
                self.current_x -= step_size
                print(f"Moving left... X = {self.current_x:.1f}")
            elif command == 'd':
                self.current_x += step_size
                print(f"Moving right... X = {self.current_x:.1f}")
            elif command == 'q':
                self.current_y += step_size
                print(f"Moving forward... Y = {self.current_y:.1f}")
            elif command == 'e':
                self.current_y -= step_size
                print(f"Moving backward... Y = {self.current_y:.1f}")
            elif command == 'n':
                break
            else:
                continue
            
            self.move_to_current_position()
        
        print("\n--- STEP 3: FINE ADJUSTMENT ---")
        print("Now fine-tune to the exact top-left corner of your paper.")
        print("Same controls, but 2mm increments.")
        print("  'f' = SAVE this position as origin")
        print("  'h' = Return HOME (abort)")
        
        step_size = 2.0
        
        while True:
            print(f"\nCurrent: X={self.current_x:.1f}, Y={self.current_y:.1f}, Z={self.current_z:.1f}")
            command = input("Command: ").lower().strip()
            
            if command == 'w':
                self.current_z += step_size
                print(f"Moving up... Z = {self.current_z:.1f}")
            elif command == 's':
                self.current_z -= step_size
                print(f"Moving down... Z = {self.current_z:.1f}")
            elif command == 'a':
                self.current_x -= step_size
                print(f"Moving left... X = {self.current_x:.1f}")
            elif command == 'd':
                self.current_x += step_size
                print(f"Moving right... X = {self.current_x:.1f}")
            elif command == 'q':
                self.current_y += step_size
                print(f"Moving forward... Y = {self.current_y:.1f}")
            elif command == 'e':
                self.current_y -= step_size
                print(f"Moving backward... Y = {self.current_y:.1f}")
            elif command == 'f':
                print("\n=== CALIBRATION COMPLETE ===")
                print("\nYour origin coordinates are:")
                print(f"  ORIGIN_X = {self.current_x:.1f}")
                print(f"  ORIGIN_Y = {self.current_y:.1f}")
                print(f"  ORIGIN_Z = {self.current_z:.1f}")
                print("\nUpdate these values in your main drawing script.")
                
                # Test the corners
                self.test_drawing_area()
                return
            elif command == 'h':
                print("Calibration aborted.")
                self.go_to_home()
                return
            else:
                continue
            
            self.move_to_current_position()
    
    def test_drawing_area(self):
        """Test the drawing area dimensions."""
        print("\n--- TESTING DRAWING AREA ---")
        print("Let's verify your drawing area dimensions.")
        
        width = float(input("Enter paper WIDTH in mm (e.g., 120): "))
        height = float(input("Enter paper HEIGHT in mm (e.g., 180): "))
        
        print("\nI'll now trace the corners of your drawing area.")
        print("Make sure the pen follows the paper edges correctly.")
        input("Press ENTER to start test...")
        
        # Define corners
        corners = [
            (self.current_x, self.current_z),  # Top-left (origin)
            (self.current_x + width, self.current_z),  # Top-right
            (self.current_x + width, self.current_z - height),  # Bottom-right
            (self.current_x, self.current_z - height),  # Bottom-left
        ]
        
        # Visit each corner
        for i, (x, z) in enumerate(corners):
            corner_names = ["TOP-LEFT (origin)", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"]
            print(f"Moving to {corner_names[i]}...")
            self.mc.send_coords([x, self.current_y, z] + self.ORIENTATION, 25, 0)
            time.sleep(2)
            
            response = input("Is this corner correct? (y/n): ").lower()
            if response != 'y':
                print("Please re-run calibration to adjust.")
                break
        
        print("\nReturning home...")
        self.go_to_home()
        
        print("\n=== SETUP COMPLETE ===")
        print("\nAdd these values to your drawing script:")
        print(f"  ORIGIN_X = {self.current_x:.1f}")
        print(f"  ORIGIN_Y = {self.current_y:.1f}")
        print(f"  ORIGIN_Z = {self.current_z:.1f}")
        print(f"  DRAWING_AREA_WIDTH_MM = {width}")
        print(f"  DRAWING_AREA_HEIGHT_MM = {height}")

if __name__ == "__main__":
    calibrator = OriginCalibrator()
    calibrator.calibrate()