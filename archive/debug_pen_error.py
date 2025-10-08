#!/usr/bin/env python3
"""
Debug script to identify the pen_is_down attribute error
"""

import sys
import os

# Add debug info
print("=== DEBUG INFO ===")
print(f"Current working directory: {os.getcwd()}")
print(f"Python path: {sys.path[0]}")
print(f"Script location: {__file__}")

# Try to import without robot connection
try:
    # Mock the robot connection to avoid hardware dependency
    class MockMyCobot320:
        def __init__(self, port, baud):
            print(f"Mock robot: {port} @ {baud}")
        def set_fresh_mode(self, mode):
            pass
    
    # Replace the real robot with mock
    import sys
    sys.modules['pymycobot'] = type('MockModule', (), {'MyCobot320': MockMyCobot320})()
    
    print("\n=== TESTING VERTICAL DRAWING ROBOT ===")
    
    # Now try to import and create the robot
    if os.path.exists('vertical_drawing_improved.py'):
        print("✓ Found vertical_drawing_improved.py")
        
        # Import the module
        import vertical_drawing_improved
        
        # Check if class exists
        if hasattr(vertical_drawing_improved, 'VerticalDrawingRobot'):
            print("✓ Found VerticalDrawingRobot class")
            
            # Try to create instance
            try:
                robot = vertical_drawing_improved.VerticalDrawingRobot()
                print("✓ Successfully created robot instance")
                
                # Check attributes
                if hasattr(robot, 'pen_is_down'):
                    print(f"✓ pen_is_down attribute exists: {robot.pen_is_down}")
                else:
                    print("✗ pen_is_down attribute missing!")
                    
                if hasattr(robot, 'current_position'):
                    print(f"✓ current_position attribute exists: {robot.current_position}")
                else:
                    print("✗ current_position attribute missing!")
                    
            except Exception as e:
                print(f"✗ Failed to create robot instance: {e}")
                import traceback
                traceback.print_exc()
        else:
            print("✗ VerticalDrawingRobot class not found")
    else:
        print("✗ vertical_drawing_improved.py not found")
        
    print("\n=== TESTING HORIZONTAL TABLE ROBOT ===")
    
    if os.path.exists('horizontal_table_drawing.py'):
        print("✓ Found horizontal_table_drawing.py")
        
        import horizontal_table_drawing
        
        if hasattr(horizontal_table_drawing, 'HorizontalTableDrawingRobot'):
            print("✓ Found HorizontalTableDrawingRobot class")
            
            try:
                robot2 = horizontal_table_drawing.HorizontalTableDrawingRobot()
                print("✓ Successfully created horizontal robot instance")
                
                if hasattr(robot2, 'pen_is_down'):
                    print(f"✓ pen_is_down attribute exists: {robot2.pen_is_down}")
                else:
                    print("✗ pen_is_down attribute missing!")
                    
            except Exception as e:
                print(f"✗ Failed to create horizontal robot: {e}")
        else:
            print("✗ HorizontalTableDrawingRobot class not found")
    else:
        print("✗ horizontal_table_drawing.py not found")

except Exception as e:
    print(f"Overall error: {e}")
    import traceback
    traceback.print_exc()

print("\n=== RECOMMENDATIONS ===")
print("1. Use horizontal_table_drawing.py (newest version)")
print("2. Make sure you're running from the correct directory")
print("3. Check if you have any __pycache__ folders to clear")
print("4. Restart your Python interpreter")