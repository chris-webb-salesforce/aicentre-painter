"""
Diagnostic tool to test and map the robot's reachable workspace
"""

from pymycobot import MyCobot320
import time
import numpy as np

class WorkspaceDiagnostic:
    def __init__(self, port="/dev/ttyAMA0", baudrate=115200):
        self.mc = MyCobot320(port, baudrate)
        time.sleep(2)
        
        # MyCobot 320 specifications
        self.JOINT_LIMITS = {
            'J1': (-165, 165),
            'J2': (-165, 165),
            'J3': (-165, 165),
            'J4': (-165, 165),
            'J5': (-165, 165),
            'J6': (-175, 175)
        }
        
        # Known good positions from your working code
        self.KNOWN_GOOD = {
            'origin': [180.0, -80.0, 120.0],
            'orientation': [90, -90, 135]  # 90° left rotation, 135° pen holder
        }
    
    def test_position(self, x, y, z, rx=180, ry=-90, rz=-90, timeout=3):
        """Test if a position is reachable and return diagnostic info."""
        print(f"\nTesting position: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
        print(f"Orientation: RX={rx}, RY={ry}, RZ={rz}")
        
        # Get current position
        start_coords = self.mc.get_coords()
        start_joints = self.mc.get_angles()
        
        print("Current state:")
        if start_coords:
            print(f"  Position: {start_coords[:3]}")
        if start_joints:
            print(f"  Joints: {[f'{j:.1f}°' for j in start_joints]}")
        
        # Attempt move
        target = [x, y, z, rx, ry, rz]
        self.mc.send_coords(target, 30, 0)
        time.sleep(timeout)
        
        # Check result
        final_coords = self.mc.get_coords()
        final_joints = self.mc.get_angles()
        
        if final_coords:
            actual = final_coords[:3]
            error = np.sqrt(sum((a - t)**2 for a, t in zip(actual, [x, y, z])))
            
            print(f"\nResult:")
            print(f"  Target: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
            print(f"  Actual: X={actual[0]:.1f}, Y={actual[1]:.1f}, Z={actual[2]:.1f}")
            print(f"  Error: {error:.1f}mm")
            
            if final_joints:
                print(f"  Joint angles: {[f'{j:.1f}°' for j in final_joints]}")
                
                # Check if any joints are at limits
                for i, (joint, angle) in enumerate(zip(self.JOINT_LIMITS.keys(), final_joints)):
                    min_limit, max_limit = self.JOINT_LIMITS[joint]
                    if abs(angle - min_limit) < 5 or abs(angle - max_limit) < 5:
                        print(f"  WARNING: {joint} near limit ({angle:.1f}° / limits: {min_limit} to {max_limit})")
            
            return error < 10  # Consider successful if within 10mm
        
        print("  ERROR: Could not read final position")
        return False
    
    def find_reachable_workspace(self):
        """Test various positions to map reachable workspace."""
        print("\n=== WORKSPACE MAPPING ===\n")
        
        # Start from known good position
        base_x, base_y, base_z = self.KNOWN_GOOD['origin']
        rx, ry, rz = self.KNOWN_GOOD['orientation']
        
        # Test different offsets
        test_offsets = [
            ("Current origin", 0, 0, 0),
            ("10mm forward (+Y)", 0, 10, 0),
            ("10mm back (-Y)", 0, -10, 0),
            ("20mm back (-Y)", 0, -20, 0),
            ("10mm right (+X)", 10, 0, 0),
            ("10mm left (-X)", -10, 0, 0),
            ("10mm up (+Z)", 0, 0, 10),
            ("10mm down (-Z)", 0, 0, -10),
            ("Corner: +X +Z", 60, 0, -90),
            ("Corner: -X -Z", -60, 0, 90),
        ]
        
        results = []
        for name, dx, dy, dz in test_offsets:
            print(f"\nTest: {name}")
            success = self.test_position(
                base_x + dx, 
                base_y + dy, 
                base_z + dz,
                rx, ry, rz
            )
            results.append((name, success))
            time.sleep(1)
        
        print("\n=== SUMMARY ===")
        for name, success in results:
            status = "✓ REACHABLE" if success else "✗ UNREACHABLE"
            print(f"{name}: {status}")
    
    def test_orientations(self, x=205.1, y=-85.3, z=110.2):
        """Test different orientations at a fixed position."""
        print(f"\n=== TESTING ORIENTATIONS at X={x}, Y={y}, Z={z} ===\n")
        
        orientations = [
            ("Original", 180, -90, -90),
            ("Alternative 1", 180, 0, -90),
            ("Alternative 2", 180, -45, -90),
            ("Alternative 3", 90, -90, 0),
            ("Alternative 4", 180, -90, 0),
            ("Straight down", 180, 0, 0),
        ]
        
        for name, rx, ry, rz in orientations:
            print(f"\nTesting {name}: RX={rx}, RY={ry}, RZ={rz}")
            success = self.test_position(x, y, z, rx, ry, rz)
            if success:
                print(f"  ✓ Orientation {name} works!")
                return (rx, ry, rz)
            time.sleep(1)
        
        return None
    
    def calibrate_drawing_position(self):
        """Interactive calibration to find optimal drawing position."""
        print("\n=== INTERACTIVE CALIBRATION ===")
        print("Let's find a working drawing position\n")
        
        # Start at home
        self.mc.send_angles([0, 0, 0, 0, 90, 0], 40)
        time.sleep(3)
        
        # Try to reach a reasonable drawing position
        # Adjusted for better reachability
        test_positions = [
            # (X, Y, Z, RX, RY, RZ)
            (200, -100, 150, 180, 0, -90),  # Higher and closer
            (180, -80, 120, 180, 0, -90),   # Even closer
            (160, -60, 100, 180, 0, -90),   # Much closer
            (220, -120, 180, 180, 0, -90),  # Further out
        ]
        
        for x, y, z, rx, ry, rz in test_positions:
            print(f"\nTrying position: X={x}, Y={y}, Z={z}")
            if self.test_position(x, y, z, rx, ry, rz):
                print("\n✓ Found working position!")
                print(f"Update your code with:")
                print(f"  ORIGIN_X = {x}")
                print(f"  ORIGIN_Y = {y}")
                print(f"  ORIGIN_Z = {z}")
                print(f"  DRAWING_ORIENTATION = [{rx}, {ry}, {rz}]")
                return (x, y, z, rx, ry, rz)
        
        return None
    
    def check_current_position(self):
        """Check the robot's current position and joint states."""
        print("\n=== CURRENT ROBOT STATE ===\n")
        
        coords = self.mc.get_coords()
        joints = self.mc.get_angles()
        
        if coords:
            print("Cartesian coordinates:")
            print(f"  X: {coords[0]:.1f}mm")
            print(f"  Y: {coords[1]:.1f}mm")
            print(f"  Z: {coords[2]:.1f}mm")
            if len(coords) > 3:
                print(f"  RX: {coords[3]:.1f}°")
                print(f"  RY: {coords[4]:.1f}°")
                print(f"  RZ: {coords[5]:.1f}°")
            
            # Calculate distance from base
            distance = np.sqrt(coords[0]**2 + coords[1]**2 + coords[2]**2)
            print(f"\nDistance from base: {distance:.1f}mm")
            print(f"Max reach of MyCobot320: ~320mm")
            
            if distance > 300:
                print("⚠️  WARNING: Position is near maximum reach!")
        
        if joints:
            print("\nJoint angles:")
            for i, (joint, angle) in enumerate(zip(self.JOINT_LIMITS.keys(), joints)):
                min_limit, max_limit = self.JOINT_LIMITS[joint]
                print(f"  {joint}: {angle:.1f}° (limits: {min_limit} to {max_limit})")
                
                if abs(angle - min_limit) < 10 or abs(angle - max_limit) < 10:
                    print(f"    ⚠️  WARNING: Near limit!")

def main():
    print("MyCobot320 Workspace Diagnostic Tool")
    print("=" * 40)
    
    diag = WorkspaceDiagnostic()
    
    while True:
        print("\nOptions:")
        print("1. Check current position")
        print("2. Test workspace mapping")
        print("3. Test different orientations")
        print("4. Calibrate drawing position")
        print("5. Test specific position")
        print("6. Home robot")
        print("0. Exit")
        
        choice = input("\nSelect option: ").strip()
        
        if choice == "1":
            diag.check_current_position()
        elif choice == "2":
            diag.find_reachable_workspace()
        elif choice == "3":
            result = diag.test_orientations()
            if result:
                print(f"\nBest orientation found: {result}")
        elif choice == "4":
            result = diag.calibrate_drawing_position()
            if not result:
                print("\nCould not find suitable position. Try moving robot closer to target.")
        elif choice == "5":
            try:
                x = float(input("Enter X: "))
                y = float(input("Enter Y: "))
                z = float(input("Enter Z: "))
                rx = float(input("Enter RX (default 180): ") or "180")
                ry = float(input("Enter RY (default 0): ") or "0")
                rz = float(input("Enter RZ (default -90): ") or "-90")
                diag.test_position(x, y, z, rx, ry, rz)
            except ValueError:
                print("Invalid input")
        elif choice == "6":
            print("Moving to home...")
            diag.mc.send_angles([0, 0, 0, 0, 90, 0], 40)
            time.sleep(3)
        elif choice == "0":
            break

if __name__ == "__main__":
    main()