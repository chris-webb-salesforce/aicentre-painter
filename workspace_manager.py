"""
Workspace and coordinate system management.
Handles home position recording, coordinate validation, and workspace calibration.
"""

import os
import json
import time
import numpy as np
from datetime import datetime
from config import *


class WorkspaceManager:
    """Manage workspace calibration and coordinate systems."""
    
    def __init__(self, robot_controller):
        """Initialize workspace manager with robot controller."""
        self.robot = robot_controller

    def validate_trajectory_points(self, contours):
        """Validate that all trajectory points are within safe bounds."""
        print("\n--- TRAJECTORY VALIDATION ---")
        
        total_points = 0
        out_of_bounds = 0
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')
        
        # Define safe workspace bounds (with safety margins)
        workspace_min_x = ORIGIN_X - 10  # 10mm safety margin
        workspace_max_x = ORIGIN_X + DRAWING_AREA_WIDTH_MM + 10
        workspace_min_y = ORIGIN_Y - 10
        workspace_max_y = ORIGIN_Y + DRAWING_AREA_HEIGHT_MM + 10
        
        for i, contour_points in enumerate(contours):
            for j, (x, y) in enumerate(contour_points):
                total_points += 1
                
                # Track actual bounds
                min_x, max_x = min(min_x, x), max(max_x, x)
                min_y, max_y = min(min_y, y), max(max_y, y)
                
                # Check if point is out of bounds
                if (x < workspace_min_x or x > workspace_max_x or 
                    y < workspace_min_y or y > workspace_max_y):
                    out_of_bounds += 1
                    print(f"⚠️  Out of bounds: Contour {i+1}, Point {j+1}: ({x:.2f}, {y:.2f})")
        
        print(f"✓ Total trajectory points: {total_points}")
        print(f"✓ Actual bounds: X=[{min_x:.1f}, {max_x:.1f}] Y=[{min_y:.1f}, {max_y:.1f}]")
        print(f"✓ Safe workspace: X=[{workspace_min_x:.1f}, {workspace_max_x:.1f}] Y=[{workspace_min_y:.1f}, {workspace_max_y:.1f}]")
        
        if out_of_bounds > 0:
            print(f"❌ WARNING: {out_of_bounds} points out of bounds!")
            return False
        else:
            print("✅ All points within safe workspace bounds")
            return True

    def verify_coordinate_transformation(self):
        """Verify pixel to mm coordinate transformation is correct."""
        print("\n--- COORDINATE TRANSFORMATION TEST ---")
        
        # Test corner points
        test_points_px = [
            (0, 0),  # Top-left pixel
            (IMAGE_WIDTH_PX-1, 0),  # Top-right pixel
            (IMAGE_WIDTH_PX-1, IMAGE_HEIGHT_PX-1),  # Bottom-right pixel
            (0, IMAGE_HEIGHT_PX-1),  # Bottom-left pixel
            (IMAGE_WIDTH_PX//2, IMAGE_HEIGHT_PX//2),  # Center pixel
        ]
        
        print("Pixel → MM → Pixel conversion test:")
        print("Original Pixel → MM Coords → Back to Pixel")
        
        for px_x, px_y in test_points_px:
            # Convert pixel to mm (same as in preprocess_contours)
            mm_x = ORIGIN_X + (px_x / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
            mm_y = ORIGIN_Y + DRAWING_AREA_HEIGHT_MM - (px_y / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
            
            # Convert back to pixel (same as in create_contour_preview)
            back_px_x = int((mm_x - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX)
            back_px_y = int(IMAGE_HEIGHT_PX - (mm_y - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM * IMAGE_HEIGHT_PX)
            
            error_x = abs(px_x - back_px_x)
            error_y = abs(px_y - back_px_y)
            
            print(f"({px_x:3d}, {px_y:3d}) → ({mm_x:6.1f}, {mm_y:6.1f}) → ({back_px_x:3d}, {back_px_y:3d}) [Error: {error_x}, {error_y}]")
        
        # Test physical workspace corners
        print("\nPhysical workspace corners (MM coordinates):")
        corners_mm = [
            (ORIGIN_X, ORIGIN_Y),  # Origin
            (ORIGIN_X + DRAWING_AREA_WIDTH_MM, ORIGIN_Y),  # Top-right
            (ORIGIN_X + DRAWING_AREA_WIDTH_MM, ORIGIN_Y + DRAWING_AREA_HEIGHT_MM),  # Bottom-right  
            (ORIGIN_X, ORIGIN_Y + DRAWING_AREA_HEIGHT_MM),  # Bottom-left
        ]
        
        for i, (mm_x, mm_y) in enumerate(corners_mm):
            print(f"Corner {i+1}: ({mm_x:6.1f}, {mm_y:6.1f}) mm")

    def create_trajectory_summary(self, contours):
        """Create a summary of the trajectory for review."""
        print("\n" + "="*50)
        print("TRAJECTORY SUMMARY")
        print("="*50)
        
        total_segments = sum(len(c)-1 for c in contours if len(c) > 1)
        total_distance = 0
        max_jump = 0
        
        for i, contour_points in enumerate(contours):
            contour_distance = 0
            if len(contour_points) > 1:
                for j in range(1, len(contour_points)):
                    x1, y1 = contour_points[j-1]
                    x2, y2 = contour_points[j]
                    seg_dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    contour_distance += seg_dist
                
                total_distance += contour_distance
                print(f"Contour {i+1:3d}: {len(contour_points):3d} points, {contour_distance:6.1f}mm length")
        
        # Calculate travel distances between contours
        travel_distance = 0
        for i in range(len(contours)-1):
            if len(contours[i]) > 0 and len(contours[i+1]) > 0:
                end_point = contours[i][-1]
                start_point = contours[i+1][0]
                travel_dist = np.sqrt((start_point[0] - end_point[0])**2 + 
                                     (start_point[1] - end_point[1])**2)
                travel_distance += travel_dist
                max_jump = max(max_jump, travel_dist)
        
        print("-"*50)
        print(f"Total contours:     {len(contours)}")
        print(f"Total segments:     {total_segments}")
        print(f"Drawing distance:   {total_distance:.1f}mm")
        print(f"Travel distance:    {travel_distance:.1f}mm")
        print(f"Max travel jump:    {max_jump:.1f}mm")
        print(f"Estimated time:     {(total_segments * 0.1 + len(contours) * 0.5):.1f}s")
        print("="*50)

    def test_coordinate_system(self):
        """Test coordinate system and transformations."""
        print("\n--- COORDINATE SYSTEM TEST ---")
        
        # Run coordinate transformation test
        self.verify_coordinate_transformation()
        
        # Test a few sample points
        print("\nTesting sample trajectory points:")
        test_contours = [
            [(ORIGIN_X + 10, ORIGIN_Y + 10), (ORIGIN_X + 30, ORIGIN_Y + 20)],  # Small line
            [(ORIGIN_X + 50, ORIGIN_Y + 50), (ORIGIN_X + 70, ORIGIN_Y + 70)],  # Diagonal line
        ]
        
        if self.validate_trajectory_points(test_contours):
            print("✅ Coordinate system test passed")
        else:
            print("❌ Coordinate system test failed")
        
        # Show trajectory summary for test points
        self.create_trajectory_summary(test_contours)

    def test_drawing_area(self):
        """Test the drawing area with gentle movements."""
        print("\n--- TESTING DRAWING AREA ---")
        print("This will draw a test rectangle to verify settings.")
        
        self.robot.go_to_home_position()
        
        # Define test rectangle (smaller than full area)
        margin = 20
        test_corners = [
            (ORIGIN_X + margin, ORIGIN_Y + margin),
            (ORIGIN_X + DRAWING_AREA_WIDTH_MM - margin, ORIGIN_Y + margin),
            (ORIGIN_X + DRAWING_AREA_WIDTH_MM - margin, ORIGIN_Y + DRAWING_AREA_HEIGHT_MM - margin),
            (ORIGIN_X + margin, ORIGIN_Y + DRAWING_AREA_HEIGHT_MM - margin),
            (ORIGIN_X + margin, ORIGIN_Y + margin),  # Close the rectangle
        ]
        
        # Move to start position
        print("Moving to start position...")
        start_x, start_y = test_corners[0]
        self.robot.synchronized_move([start_x, start_y, PEN_RETRACT_Z] + DRAWING_ORIENTATION, TRAVEL_SPEED, 0)
        
        # Draw test rectangle
        print("Drawing test rectangle...")
        self.robot.gentle_pen_down(start_x, start_y)
        
        for i in range(1, len(test_corners)):
            print(f"Drawing edge {i}/{len(test_corners)-1}...")
            self.robot.draw_line_segment(test_corners[i-1], test_corners[i])
            time.sleep(0.2)
        
        self.robot.gentle_pen_up()
        print("Test complete!")
        self.robot.go_to_home_position()


class HomePositionManager:
    """Manage home position recording and loading for ElephantRobotics workflow."""
    
    def __init__(self, robot_controller):
        """Initialize with robot controller."""
        self.robot = robot_controller

    def record_home_position(self):
        """
        Record current robot position as home position (as described in their workflow).
        This should be done after manually positioning the pen tip on the drawing board.
        """
        print("\n--- RECORDING HOME POSITION ---")
        print("Position the robot arm so the pen tip lightly touches the drawing board,")
        print("then press Enter to record this as the home position.")
        input("Press Enter when ready...")
        
        try:
            # Get current joint angles and coordinates
            current_angles = self.robot.mc.get_angles()
            current_coords = self.robot.mc.get_coords()
            
            if not current_angles or not current_coords:
                print("❌ Failed to get current robot position")
                return False
            
            home_position = {
                "joint_angles": current_angles,
                "cartesian_coords": current_coords,
                "timestamp": datetime.now().isoformat(),
                "description": "Home position for drawing board contact"
            }
            
            # Save to file
            with open(HOME_POSITION_FILE, 'w') as f:
                json.dump(home_position, f, indent=2)
            
            print("✅ Home position recorded successfully!")
            print(f"Joint angles: {current_angles}")
            print(f"Cartesian coordinates: {current_coords}")
            print(f"Saved to: {HOME_POSITION_FILE}")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to record home position: {e}")
            return False

    def load_home_position(self):
        """Load previously recorded home position."""
        try:
            if not os.path.exists(HOME_POSITION_FILE):
                print(f"❌ Home position file not found: {HOME_POSITION_FILE}")
                return None
            
            with open(HOME_POSITION_FILE, 'r') as f:
                home_position = json.load(f)
            
            print(f"✅ Home position loaded from {HOME_POSITION_FILE}")
            print(f"Recorded: {home_position.get('timestamp', 'Unknown')}")
            
            return home_position
            
        except Exception as e:
            print(f"❌ Failed to load home position: {e}")
            return None

    def go_to_recorded_home(self):
        """Move to the recorded home position."""
        home_pos = self.load_home_position()
        if not home_pos:
            print("Using default home position instead...")
            self.robot.go_to_home_position()
            return False
        
        try:
            print("Moving to recorded home position...")
            
            # Move to recorded joint angles
            joint_angles = home_pos["joint_angles"]
            self.robot.synchronized_angles(joint_angles, 30)
            
            print("✅ Moved to recorded home position")
            return True
            
        except Exception as e:
            print(f"❌ Failed to move to recorded home position: {e}")
            print("Using default home position instead...")
            self.robot.go_to_home_position()
            return False

    def calibrate_pen_pressure(self):
        """Interactive calibration to find optimal pen pressure."""
        print("\n--- PEN PRESSURE CALIBRATION ---")
        print("This will help find the optimal pen pressure for your setup.")
        print("Place a test paper on the drawing surface.")
        
        self.robot.go_to_home_position()
        
        # Move to center of drawing area
        test_x = ORIGIN_X + DRAWING_AREA_WIDTH_MM / 2
        test_z = ORIGIN_Z - DRAWING_AREA_HEIGHT_MM / 2
        test_y = ORIGIN_Y
        step_size = 0.5
        
        print("Moving to test position...")
        self.robot.synchronized_move([test_x, test_y, PEN_RETRACT_Z] + DRAWING_ORIENTATION, 30, 0)
        time.sleep(3)
        
        print("\nUse keyboard to adjust pen pressure:")
        print("  'f' = Move pen forward (more pressure)")
        print("  'b' = Move pen backward (less pressure)")
        print("  's' = Save this position")
        print("  'q' = Cancel calibration")
        
        while True:
            print(f"Current Y position: {test_y:.1f}mm")
            self.robot.synchronized_move([test_x, test_y, test_z] + DRAWING_ORIENTATION, 15, 0)
            time.sleep(0.5)
            
            key = input("Command: ").lower().strip()
            
            if key == 'f':
                test_y += step_size
                print("Moving forward...")
            elif key == 'b':
                test_y -= step_size
                print("Moving backward...")
            elif key == 's':
                print(f"\nOptimal pressure Y position saved: {test_y:.1f}mm")
                print("Update PEN_DRAWING_Y in your code to this value.")
                self.robot.synchronized_move([test_x, test_y, PEN_RETRACT_Z] + DRAWING_ORIENTATION, 30, 0)
                time.sleep(2)
                return test_y
            elif key == 'q':
                print("Calibration cancelled.")
                self.robot.synchronized_move([test_x, test_y, PEN_RETRACT_Z] + DRAWING_ORIENTATION, 30, 0)
                time.sleep(2)
                return None