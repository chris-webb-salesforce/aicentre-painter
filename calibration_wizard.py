import sys
import tty
import termios
import json
import time
from pymycobot import MyCobot320
from config import * # Imports your PORT and BAUD

# --- SETTINGS ---
MOVE_SPEED = 30
STEP_LARGE = 10.0  # mm
STEP_SMALL = 1.0   # mm
STEP_MICRO = 0.5   # mm for final touch

class CalibrationWizard:
    def __init__(self):
        print("Connecting to robot...")
        self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
        time.sleep(1)
        self.mc.power_on()
        self.step_size = STEP_LARGE
        
        # Storage for the 3 points
        self.points = {
            "p1": None, # Top Left
            "p2": None, # Top Right
            "p3": None  # Bottom Left
        }

    def get_key(self):
        """Reads a single keypress without waiting for Enter"""
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
        if not coords: return
        
        print("\033[H\033[J") # Clear screen
        print(f"--- ROBOT ARTIST CALIBRATION WIZARD ---")
        print(f"Current Pos: X={coords[0]:.1f} Y={coords[1]:.1f} Z={coords[2]:.1f}")
        print(f"Orientation: Rx={coords[3]:.1f} Ry={coords[4]:.1f} Rz={coords[5]:.1f}")
        print(f"Step Size: {self.step_size}mm")
        print("-" * 40)
        print("CONTROLS:")
        print("  W / S :  Move Up / Down (X axis usually)")
        print("  A / D :  Move Left / Right (Y axis usually)")
        print("  Q / E :  Move In / Out (Z axis - towards wall)")
        print("  R / F :  Tilt Pen Pitch (Adjust Angle)")
        print("-" * 40)
        print("  + / - :  Increase/Decrease Step Size")
        print("-" * 40)
        print(f"  1 : Save TOP-LEFT     [{'DONE' if self.points['p1'] else 'Pending'}]")
        print(f"  2 : Save TOP-RIGHT    [{'DONE' if self.points['p2'] else 'Pending'}]")
        print(f"  3 : Save BOTTOM-LEFT  [{'DONE' if self.points['p3'] else 'Pending'}]")
        print("  X : Save & Exit")
        print("  Ctrl+C to Quit")

    def run(self):
        print("Starting Wizard...")
        # Initial safe move to get coords
        current_coords = self.mc.get_coords()
        if not current_coords:
            print("Error: Could not read robot coordinates. Check connection.")
            return

        while True:
            self.print_status()
            key = self.get_key().lower()

            # Read fresh coords every loop
            coords = self.mc.get_coords()
            if not coords: continue
            
            x, y, z, rx, ry, rz = coords

            # --- MOVEMENT LOGIC ---
            # Note: Depending on your mounting (Table vs Wall), X and Z might swap roles.
            # Adjust these mappings if W makes the robot go sideways.
            
            if key == 'w': x += self.step_size   # Up
            elif key == 's': x -= self.step_size # Down
            elif key == 'a': y += self.step_size # Left
            elif key == 'd': y -= self.step_size # Right
            
            elif key == 'q': z -= self.step_size # In (Towards Wall)
            elif key == 'e': z += self.step_size # Out (Away)
            
            # --- ORIENTATION LOGIC ---
            # Adjust Rx to get pen perpendicular
            elif key == 'r': rx += 2.0 
            elif key == 'f': rx -= 2.0
            
            # --- STEP SIZE ---
            elif key == '+': self.step_size = STEP_LARGE
            elif key == '-': self.step_size = STEP_SMALL
            
            # --- SAVING POINTS ---
            elif key == '1': 
                self.points['p1'] = coords
                print("\nSaved Top-Left!")
                time.sleep(0.5)
            elif key == '2': 
                self.points['p2'] = coords
                print("\nSaved Top-Right!")
                time.sleep(0.5)
            elif key == '3': 
                self.points['p3'] = coords
                print("\nSaved Bottom-Left!")
                time.sleep(0.5)
                
            # --- FINISH ---
            elif key == 'x':
                if all(self.points.values()):
                    self.save_to_file()
                    break
                else:
                    print("\nError: Please save all 3 points first.")
                    time.sleep(1)
            
            elif ord(key) == 3: # Ctrl+C
                break

            # Execute the move
            # Check bounds roughly (software limit)
            self.mc.send_coords([x, y, z, rx, ry, rz], MOVE_SPEED, 0)
            time.sleep(0.1)

    def save_to_file(self):
        # Calculate average orientation for drawing consistency
        p1 = self.points['p1']
        orientation = [p1[3], p1[4], p1[5]] # Use P1's orientation as the master
        
        data = {
            "p1": self.points['p1'],
            "p2": self.points['p2'],
            "p3": self.points['p3'],
            "orientation": orientation
        }
        
        with open("surface_calibration.json", 'w') as f:
            json.dump(data, f, indent=4)
        print("\n\nCALIBRATION SAVED SUCCESSFULLY! Exiting...")

if __name__ == "__main__":
    wizard = CalibrationWizard()
    wizard.run()
