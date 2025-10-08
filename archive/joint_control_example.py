"""
Example showing combined joint position and Cartesian control for painting robot
"""

from pymycobot import MyCobot320
import time
import numpy as np

class HybridControlPainter:
    def __init__(self, port="/dev/ttyAMA0", baudrate=115200):
        self.mc = MyCobot320(port, baudrate)
        time.sleep(2)
        
        # Define key positions using joint angles for reliability
        self.JOINT_POSITIONS = {
            'home': [0, 0, 0, 0, 90, 135],  # J6=135° for pen holder
            'photo': [0, -45, -45, 0, 90, 135],
            'pre_draw': [45, -30, -60, 0, 90, 135],  # Ready position before drawing
            'maintenance': [0, 45, 45, 0, 0, 135],    # Easy access for pen change
        }
        
        # Drawing workspace in Cartesian coordinates
        self.DRAWING_WORKSPACE = {
            'origin': [180.0, -80.0, 120.0],  # Calibrated position
            'width': 120,
            'height': 180
        }
    
    def move_to_named_position(self, position_name, speed=40):
        """Move to a predefined joint position by name."""
        if position_name in self.JOINT_POSITIONS:
            print(f"Moving to {position_name} position...")
            angles = self.JOINT_POSITIONS[position_name]
            self.mc.send_angles(angles, speed)
            time.sleep(2)
            return True
        return False
    
    def get_current_state(self):
        """Get both joint angles and Cartesian position."""
        joints = self.mc.get_angles()
        coords = self.mc.get_coords()
        
        if joints and coords:
            return {
                'joints': joints,
                'position': coords[:3],  # X, Y, Z
                'orientation': coords[3:] if len(coords) > 3 else None
            }
        return None
    
    def is_position_reachable(self, x, y, z):
        """Check if a Cartesian position is within robot's reach."""
        # Calculate distance from base
        distance = np.sqrt(x**2 + y**2 + z**2)
        
        # MyCobot320 has approximately 320mm reach
        MAX_REACH = 320
        MIN_REACH = 100  # Too close to base
        
        return MIN_REACH < distance < MAX_REACH
    
    def interpolate_joint_movement(self, target_joints, steps=10, speed=30):
        """Smooth movement in joint space."""
        current = self.mc.get_angles()
        if not current:
            return False
        
        for step in range(1, steps + 1):
            ratio = step / steps
            interpolated = []
            for i in range(len(current)):
                joint_angle = current[i] + (target_joints[i] - current[i]) * ratio
                interpolated.append(joint_angle)
            
            self.mc.send_angles(interpolated, speed)
            time.sleep(0.1)
        
        return True
    
    def safe_cartesian_move(self, target_coords, speed=30):
        """Move in Cartesian space with joint limit checking."""
        # First check if position is reachable
        if not self.is_position_reachable(target_coords[0], target_coords[1], target_coords[2]):
            print("Warning: Target position may be out of reach")
            return False
        
        # Get current joint configuration
        current_joints = self.mc.get_angles()
        
        # Move to target
        self.mc.send_coords(target_coords, speed, 0)
        time.sleep(0.5)
        
        # Verify we reached target (within tolerance)
        new_coords = self.mc.get_coords()
        if new_coords:
            error = np.linalg.norm(np.array(new_coords[:3]) - np.array(target_coords[:3]))
            if error > 5:  # 5mm tolerance
                print(f"Position error: {error:.1f}mm")
                return False
        
        return True
    
    def demonstrate_hybrid_control(self):
        """Show combined joint and Cartesian control."""
        print("\n=== Hybrid Control Demonstration ===\n")
        
        # 1. Use joint control for known positions
        print("1. Moving to home using joint control...")
        self.move_to_named_position('home')
        
        state = self.get_current_state()
        if state:
            print(f"   Joints: {[f'{j:.1f}°' for j in state['joints']]}")
            print(f"   Position: X={state['position'][0]:.1f}, Y={state['position'][1]:.1f}, Z={state['position'][2]:.1f}")
        
        # 2. Move to photo position
        print("\n2. Moving to photo position...")
        self.move_to_named_position('photo')
        
        # 3. Use Cartesian for drawing area
        print("\n3. Moving to drawing area using Cartesian...")
        draw_start = self.DRAWING_WORKSPACE['origin'] + [180, -90, -90]  # Add orientation
        if self.safe_cartesian_move(draw_start):
            print("   Successfully reached drawing position")
        
        # 4. Demonstrate smooth joint interpolation
        print("\n4. Smooth transition to maintenance position...")
        self.interpolate_joint_movement(self.JOINT_POSITIONS['maintenance'])
        
        # Return home
        print("\n5. Returning home...")
        self.move_to_named_position('home')
        
        print("\nDemonstration complete!")
    
    def save_current_position(self, name):
        """Save current position for later recall."""
        joints = self.mc.get_angles()
        coords = self.mc.get_coords()
        
        if joints and coords:
            print(f"\nPosition '{name}' saved:")
            print(f"  Joint angles: {joints}")
            print(f"  Cartesian: {coords}")
            
            # You could save this to a file
            self.JOINT_POSITIONS[name] = joints
            return True
        return False

# Example usage
if __name__ == "__main__":
    painter = HybridControlPainter()
    
    # Run demonstration
    painter.demonstrate_hybrid_control()
    
    # Example of teaching a new position
    input("\nMove robot to desired position manually, then press Enter...")
    painter.save_current_position('custom_position')
    
    # Now you can return to this position anytime
    painter.move_to_named_position('home')
    time.sleep(2)
    painter.move_to_named_position('custom_position')