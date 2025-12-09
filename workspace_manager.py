"""
Workspace and coordinate system management.
"""

import os
import json
import numpy as np
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
            for j, point in enumerate(contour_points):
                x, y = point[0], point[1]
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
                    x1, y1 = contour_points[j-1][0], contour_points[j-1][1]
                    x2, y2 = contour_points[j][0], contour_points[j][1]
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


class SurfaceCalibrator:
    def __init__(self):
        self.plane_coeffs = None
        self.calibration_data = {}
        self.load_calibration()

    def calculate_plane(self, p1, p2, p3):
        """Calculate plane equation Ax + By + Cz + D = 0 from 3 points."""
        p1 = np.array(p1[:3])
        p2 = np.array(p2[:3])
        p3 = np.array(p3[:3])

        v1 = p2 - p1
        v2 = p3 - p1

        normal = np.cross(v1, v2)
        a, b, c = normal
        d = -np.dot(normal, p1)

        self.plane_coeffs = (a, b, c, d)
        return (a, b, c, d)

    def get_z_at(self, x, y):
        """Returns the Z coordinate of the surface at X, Y."""
        if not self.plane_coeffs:
            print("CRITICAL: No calibration loaded. Returning safe default.")
            return SAFE_TRAVEL_HEIGHT

        a, b, c, d = self.plane_coeffs

        if abs(c) < 0.001:
            return SAFE_TRAVEL_HEIGHT

        z = -(a * x + b * y + d) / c
        return z

    def save_calibration(self, p1, p2, p3, orientation):
        """Save the 3 calibration points and tool orientation."""
        data = {
            "p1": p1,
            "p2": p2,
            "p3": p3,
            "orientation": orientation
        }

        self.calculate_plane(p1, p2, p3)
        
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Calibration saved to {CALIBRATION_FILE}")

    def load_calibration(self):
        if not os.path.exists(CALIBRATION_FILE):
            print("No surface calibration file found.")
            return False
            
        try:
            with open(CALIBRATION_FILE, 'r') as f:
                data = json.load(f)
                
            self.calibration_data = data
            self.calculate_plane(data['p1'], data['p2'], data['p3'])
            print(f"Surface calibration loaded. Orientation: {data['orientation']}")
            return True
        except Exception as e:
            print(f"Failed to load calibration: {e}")
            return False