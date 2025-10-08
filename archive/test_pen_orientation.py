"""
Test script to verify the 45-degree pen holder orientation
"""

from pymycobot import MyCobot320
import time

# Configuration
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

# Drawing position settings from main script (calibrated values)
ORIGIN_X = 180.0
ORIGIN_Y = -80.0
ORIGIN_Z = 120.0
PEN_RETRACT_Y = ORIGIN_Y - 30

# Orientation with 135° pen holder rotation, drawing area 90° left
DRAWING_ORIENTATION = [90, -90, 135]  # RZ=135 for pen holder

def test_pen_orientation():
    """Test the pen orientation at 45 degrees."""
    print("Testing Pen Holder Orientation (45°)")
    print("=" * 40)
    
    # Initialize robot
    mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
    time.sleep(2)
    
    print("\n1. Moving to safe position first...")
    mc.send_angles([0, 0, 0, 0, 90, 135], 40)  # Note J6=135°
    time.sleep(3)
    
    print("\n2. Moving to drawing start position with 135° pen rotation...")
    drawing_start = [ORIGIN_X, PEN_RETRACT_Y, ORIGIN_Z] + DRAWING_ORIENTATION
    print(f"   Target: X={ORIGIN_X}, Y={PEN_RETRACT_Y}, Z={ORIGIN_Z}")
    print(f"   Orientation: RX=90, RY=-90, RZ=135")
    
    mc.send_coords(drawing_start, 30, 0)
    time.sleep(3)
    
    # Check actual position
    actual = mc.get_coords()
    if actual:
        print(f"\n3. Actual position reached:")
        print(f"   X={actual[0]:.1f}, Y={actual[1]:.1f}, Z={actual[2]:.1f}")
        print(f"   RX={actual[3]:.1f}, RY={actual[4]:.1f}, RZ={actual[5]:.1f}")
        
        # Calculate error
        pos_error = ((actual[0]-ORIGIN_X)**2 + (actual[1]-PEN_RETRACT_Y)**2 + (actual[2]-ORIGIN_Z)**2)**0.5
        print(f"   Position error: {pos_error:.1f}mm")
        
        if abs(actual[5] - 135) < 5:
            print("   ✓ Pen holder correctly oriented at 135°!")
        else:
            print(f"   ⚠ Pen holder rotation off by {abs(actual[5]-135):.1f}°")
    
    # Test joint positions
    joints = mc.get_angles()
    if joints:
        print(f"\n4. Joint angles:")
        joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        for name, angle in zip(joint_names, joints):
            print(f"   {name}: {angle:.1f}°")
        
        if abs(joints[5] - 135) < 5:
            print("   ✓ J6 (pen holder) correctly at 135°!")
    
    # Test small drawing movement
    print("\n5. Testing small drawing movement...")
    test_positions = [
        (ORIGIN_X + 10, PEN_RETRACT_Y, ORIGIN_Z, "10mm right"),
        (ORIGIN_X + 10, PEN_RETRACT_Y, ORIGIN_Z - 10, "10mm down"),
        (ORIGIN_X, PEN_RETRACT_Y, ORIGIN_Z - 10, "10mm left"),
        (ORIGIN_X, PEN_RETRACT_Y, ORIGIN_Z, "back to start"),
    ]
    
    for x, y, z, desc in test_positions:
        print(f"   Moving {desc}...")
        mc.send_coords([x, y, z] + DRAWING_ORIENTATION, 25, 0)
        time.sleep(1)
    
    print("\n6. Returning to safe position...")
    mc.send_angles([0, 0, 0, 0, 90, 135], 40)
    time.sleep(3)
    
    print("\n✓ Test complete!")
    print("The pen holder should maintain 135° rotation throughout.")

if __name__ == "__main__":
    test_pen_orientation()