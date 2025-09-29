"""
G-code and NGC file handling module.
Handles parsing, validation, and execution of G-code files.
"""

import os
import time
import numpy as np
from datetime import datetime
from config import *


class GCodeHandler:
    """Handle G-code and NGC file operations."""
    
    def __init__(self, robot_controller, home_position_manager):
        """Initialize with robot controller and home position manager."""
        self.robot = robot_controller
        self.home_manager = home_position_manager

    def douglas_peucker_simplify(self, points, tolerance):
        """
        Simplify a path using Douglas-Peucker algorithm for optimal accuracy.
        Reduces points while preserving path shape within tolerance.
        """
        if len(points) < 3:
            return points
            
        def perpendicular_distance(point, line_start, line_end):
            """Calculate perpendicular distance from point to line."""
            if line_start == line_end:
                return np.sqrt((point[0] - line_start[0])**2 + (point[1] - line_start[1])**2)
            
            # Vector from start to end
            line_vec = np.array([line_end[0] - line_start[0], line_end[1] - line_start[1]])
            # Vector from start to point
            point_vec = np.array([point[0] - line_start[0], point[1] - line_start[1]])
            
            # Project point onto line
            line_len_sq = np.dot(line_vec, line_vec)
            if line_len_sq == 0:
                return np.linalg.norm(point_vec)
            
            projection = np.dot(point_vec, line_vec) / line_len_sq
            projection = max(0, min(1, projection))  # Clamp to line segment
            
            # Calculate perpendicular distance
            closest_point = line_start + projection * line_vec
            return np.linalg.norm(np.array(point) - closest_point)
        
        def simplify_recursive(points, start_idx, end_idx):
            """Recursively simplify path segment."""
            if end_idx <= start_idx + 1:
                return [start_idx, end_idx]
            
            # Find point with maximum distance from line
            max_distance = 0
            max_idx = start_idx
            
            for i in range(start_idx + 1, end_idx):
                distance = perpendicular_distance(points[i], points[start_idx], points[end_idx])
                if distance > max_distance:
                    max_distance = distance
                    max_idx = i
            
            # If max distance is within tolerance, keep only endpoints
            if max_distance <= tolerance:
                return [start_idx, end_idx]
            
            # Recursively simplify both segments
            left_indices = simplify_recursive(points, start_idx, max_idx)
            right_indices = simplify_recursive(points, max_idx, end_idx)
            
            # Combine results (remove duplicate middle point)
            return left_indices[:-1] + right_indices
        
        # Apply algorithm
        keep_indices = simplify_recursive(points, 0, len(points) - 1)
        return [points[i] for i in keep_indices]

    def smooth_path(self, points, window_size=3):
        """Apply smoothing to reduce jitter while preserving accuracy."""
        if len(points) < window_size or not ENABLE_PATH_SMOOTHING:
            return points
        
        smoothed = []
        half_window = window_size // 2
        
        for i in range(len(points)):
            start_idx = max(0, i - half_window)
            end_idx = min(len(points), i + half_window + 1)
            
            # Average coordinates within window
            x_sum = sum(p[0] for p in points[start_idx:end_idx])
            y_sum = sum(p[1] for p in points[start_idx:end_idx])
            count = end_idx - start_idx
            
            smoothed.append((x_sum / count, y_sum / count))
        
        return smoothed

    def calculate_path_complexity(self, points):
        """Calculate path complexity to determine optimal speeds."""
        if len(points) < 3:
            return 0.0
        
        total_curvature = 0.0
        
        for i in range(1, len(points) - 1):
            # Calculate angle between segments
            v1 = np.array([points[i][0] - points[i-1][0], points[i][1] - points[i-1][1]])
            v2 = np.array([points[i+1][0] - points[i][0], points[i+1][1] - points[i][1]])
            
            # Normalize vectors
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm > 0 and v2_norm > 0:
                v1_unit = v1 / v1_norm
                v2_unit = v2 / v2_norm
                
                # Calculate curvature (angle change)
                dot_product = np.dot(v1_unit, v2_unit)
                dot_product = np.clip(dot_product, -1.0, 1.0)  # Prevent numerical errors
                angle = np.arccos(dot_product)
                total_curvature += angle
        
        # Return average curvature
        return total_curvature / (len(points) - 2) if len(points) > 2 else 0.0

    def optimize_feedrate_for_segment(self, start_point, end_point, path_complexity=0.0):
        """Calculate optimal feed rate based on segment characteristics."""
        if not ENABLE_ADAPTIVE_SPEED:
            return GCODE_DRAWING_FEEDRATE
        
        # Calculate segment length
        length = np.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)
        
        # Base feed rate
        base_feedrate = GCODE_DRAWING_FEEDRATE
        
        # Adjust for path complexity (higher complexity = slower speed)
        complexity_factor = 1.0 - (path_complexity * 0.5)  # Up to 50% reduction
        complexity_factor = max(0.3, complexity_factor)  # Minimum 30% of base speed
        
        # Adjust for segment length (very short segments need slower speed)
        if length < GCODE_MIN_SEGMENT_LENGTH * 2:
            length_factor = 0.7  # Slow down for very short segments
        elif length > GCODE_MAX_SEGMENT_LENGTH:
            length_factor = 1.2  # Speed up for long segments
        else:
            length_factor = 1.0
        
        # Calculate final feed rate
        optimized_feedrate = int(base_feedrate * complexity_factor * length_factor)
        
        # Clamp to reasonable bounds
        return max(100, min(optimized_feedrate, GCODE_RAPID_FEEDRATE))

    def optimize_contour_order(self, contours):
        """
        Optimize the order of contours to minimize travel distance.
        Uses nearest-neighbor algorithm with 2-opt improvements.
        """
        if len(contours) < 2:
            return contours
        
        print("Optimizing contour drawing order...")
        
        # Convert contours to start/end points for distance calculations
        contour_info = []
        for i, contour in enumerate(contours):
            if len(contour) >= 2:
                start_point = contour[0]
                end_point = contour[-1]
                contour_info.append({
                    'index': i,
                    'contour': contour,
                    'start': start_point,
                    'end': end_point,
                    'length': len(contour)
                })
        
        if len(contour_info) < 2:
            return contours
        
        def distance(point1, point2):
            """Calculate Euclidean distance between two points."""
            return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        
        # Nearest neighbor algorithm
        ordered_contours = []
        remaining = contour_info.copy()
        
        # Start with the contour closest to origin
        origin = (ORIGIN_X, ORIGIN_Y)
        current_contour = min(remaining, key=lambda c: distance(c['start'], origin))
        remaining.remove(current_contour)
        ordered_contours.append(current_contour)
        current_position = current_contour['end']
        
        # Add remaining contours in nearest-neighbor order
        while remaining:
            # Find closest contour to current position
            closest_contour = None
            min_distance = float('inf')
            
            for contour in remaining:
                # Check distance to start of contour
                dist_to_start = distance(current_position, contour['start'])
                
                if dist_to_start < min_distance:
                    min_distance = dist_to_start
                    closest_contour = contour
            
            if closest_contour:
                remaining.remove(closest_contour)
                ordered_contours.append(closest_contour)
                current_position = closest_contour['end']
        
        # Apply 2-opt improvement for better optimization
        improved_order = self.two_opt_improve(ordered_contours)
        
        # Calculate travel distance improvement
        original_distance = self.calculate_total_travel_distance(contour_info)
        optimized_distance = self.calculate_total_travel_distance(improved_order)
        improvement = ((original_distance - optimized_distance) / original_distance) * 100
        
        print(f"✅ Path optimization complete:")
        print(f"   Travel distance reduced by {improvement:.1f}%")
        print(f"   Original: {original_distance:.1f}mm")
        print(f"   Optimized: {optimized_distance:.1f}mm")
        
        # Return ordered contours
        return [info['contour'] for info in improved_order]

    def two_opt_improve(self, ordered_contours):
        """Apply 2-opt algorithm to improve contour ordering."""
        if len(ordered_contours) < 4:
            return ordered_contours
        
        improved = ordered_contours.copy()
        improved_distance = self.calculate_total_travel_distance(improved)
        
        # Try swapping pairs of contours
        for i in range(1, len(improved) - 2):
            for j in range(i + 1, len(improved)):
                if j - i == 1:
                    continue  # Skip adjacent swaps
                
                # Create new order with reversed segment
                new_order = (improved[:i] + 
                           list(reversed(improved[i:j])) + 
                           improved[j:])
                
                new_distance = self.calculate_total_travel_distance(new_order)
                
                if new_distance < improved_distance:
                    improved = new_order
                    improved_distance = new_distance
        
        return improved

    def calculate_total_travel_distance(self, contour_info_list):
        """Calculate total travel distance for a given contour order."""
        if len(contour_info_list) < 2:
            return 0.0
        
        def distance(point1, point2):
            return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        
        total_distance = 0.0
        
        # Distance from origin to first contour
        origin = (ORIGIN_X, ORIGIN_Y)
        total_distance += distance(origin, contour_info_list[0]['start'])
        
        # Distance between contours
        for i in range(len(contour_info_list) - 1):
            current_end = contour_info_list[i]['end']
            next_start = contour_info_list[i + 1]['start']
            total_distance += distance(current_end, next_start)
        
        return total_distance

    def validate_gcode_file(self, filename):
        """
        Comprehensive validation of generated G-code file.
        Checks for accuracy, safety, and best practices.
        """
        print(f"\nValidating G-code file: {filename}")
        
        issues = []
        line_count = 0
        move_commands = 0
        feedrate_changes = 0
        coordinate_bounds = {"X": [], "Y": [], "Z": []}
        current_feedrate = None
        in_precision_mode = False
        
        try:
            with open(filename, 'r') as file:
                for line_num, line in enumerate(file, 1):
                    line_count += 1
                    command = line.strip()
                    
                    # Skip comments and empty lines
                    if not command or command.startswith(';'):
                        continue
                    
                    # Check for precision mode setting
                    if "G61" in command:
                        in_precision_mode = True
                    elif "G64" in command:
                        in_precision_mode = False
                    
                    # Analyze movement commands
                    if command.startswith(("G0", "G1", "G00", "G01")):
                        move_commands += 1
                        command_parts = command.split()
                        
                        # Extract coordinates
                        for part in command_parts[1:]:
                            if part.startswith(("X", "x")):
                                try:
                                    x_val = float(part[1:])
                                    coordinate_bounds["X"].append(x_val)
                                except ValueError:
                                    issues.append(f"Line {line_num}: Invalid X coordinate: {part}")
                            elif part.startswith(("Y", "y")):
                                try:
                                    y_val = float(part[1:])
                                    coordinate_bounds["Y"].append(y_val)
                                except ValueError:
                                    issues.append(f"Line {line_num}: Invalid Y coordinate: {part}")
                            elif part.startswith(("Z", "z")):
                                try:
                                    z_val = float(part[1:])
                                    coordinate_bounds["Z"].append(z_val)
                                except ValueError:
                                    issues.append(f"Line {line_num}: Invalid Z coordinate: {part}")
                    
                    # Track feedrate changes
                    if command.startswith("F"):
                        try:
                            new_feedrate = int(command[1:])
                            if new_feedrate != current_feedrate:
                                feedrate_changes += 1
                                current_feedrate = new_feedrate
                                
                                # Validate feedrate ranges
                                if new_feedrate < 50:
                                    issues.append(f"Line {line_num}: Very slow feedrate ({new_feedrate}), may cause jerky motion")
                                elif new_feedrate > 5000:
                                    issues.append(f"Line {line_num}: Very fast feedrate ({new_feedrate}), may reduce accuracy")
                        except ValueError:
                            issues.append(f"Line {line_num}: Invalid feedrate: {command}")
                    
        except Exception as e:
            issues.append(f"Failed to read file: {e}")
            return False, issues
        
        # Validate coordinate ranges
        workspace_issues = []
        if coordinate_bounds["X"]:
            x_min, x_max = min(coordinate_bounds["X"]), max(coordinate_bounds["X"])
            if x_min < ORIGIN_X - 50 or x_max > ORIGIN_X + DRAWING_AREA_WIDTH_MM + 50:
                workspace_issues.append(f"X coordinates ({x_min:.1f} to {x_max:.1f}) may exceed safe workspace")
        
        if coordinate_bounds["Y"]:
            y_min, y_max = min(coordinate_bounds["Y"]), max(coordinate_bounds["Y"])
            if y_min < ORIGIN_Y - 50 or y_max > ORIGIN_Y + DRAWING_AREA_HEIGHT_MM + 50:
                workspace_issues.append(f"Y coordinates ({y_min:.1f} to {y_max:.1f}) may exceed safe workspace")
        
        if coordinate_bounds["Z"]:
            z_min, z_max = min(coordinate_bounds["Z"]), max(coordinate_bounds["Z"])
            if z_min < MIN_SAFE_Z:
                workspace_issues.append(f"Z minimum ({z_min:.1f}) below safety threshold ({MIN_SAFE_Z:.1f})")
            if z_max > MAX_SAFE_Z:
                workspace_issues.append(f"Z maximum ({z_max:.1f}) above safety threshold ({MAX_SAFE_Z:.1f})")
        
        issues.extend(workspace_issues)
        
        # Print validation results
        print(f"Validation Results:")
        print(f"  Lines processed: {line_count}")
        print(f"  Movement commands: {move_commands}")
        print(f"  Feedrate changes: {feedrate_changes}")
        print(f"  Precision mode: {'Enabled (G61)' if in_precision_mode else 'Disabled (G64)'}")
        
        if coordinate_bounds["X"]:
            print(f"  X range: {min(coordinate_bounds['X']):.1f} to {max(coordinate_bounds['X']):.1f} mm")
        if coordinate_bounds["Y"]:
            print(f"  Y range: {min(coordinate_bounds['Y']):.1f} to {max(coordinate_bounds['Y']):.1f} mm")
        if coordinate_bounds["Z"]:
            print(f"  Z range: {min(coordinate_bounds['Z']):.1f} to {max(coordinate_bounds['Z']):.1f} mm")
        
        if issues:
            print(f"  ⚠️  Issues found: {len(issues)}")
            for issue in issues[:5]:  # Show first 5 issues
                print(f"    - {issue}")
            if len(issues) > 5:
                print(f"    ... and {len(issues) - 5} more issues")
            return False, issues
        else:
            print("  ✅ G-code validation passed - ready for high-accuracy drawing")
            return True, []

    def analyze_gcode_accuracy(self, filename):
        """Analyze G-code file for accuracy metrics."""
        print(f"\nAnalyzing G-code accuracy metrics...")
        
        total_distance = 0.0
        total_moves = 0
        speed_changes = 0
        pause_commands = 0
        current_pos = [ORIGIN_X, ORIGIN_Y, SAFE_TRAVEL_HEIGHT]
        current_feedrate = None
        
        try:
            with open(filename, 'r') as file:
                for line in file:
                    command = line.strip()
                    
                    if command.startswith(("G0", "G1", "G00", "G01")):
                        total_moves += 1
                        next_pos = current_pos.copy()
                        
                        # Parse new position
                        command_parts = command.split()
                        for part in command_parts[1:]:
                            if part.startswith("X"):
                                next_pos[0] = float(part[1:])
                            elif part.startswith("Y"):
                                next_pos[1] = float(part[1:])
                            elif part.startswith("Z"):
                                next_pos[2] = float(part[1:])
                        
                        # Calculate distance
                        distance = np.sqrt(sum((next_pos[i] - current_pos[i])**2 for i in range(3)))
                        total_distance += distance
                        current_pos = next_pos
                    
                    elif command.startswith("F"):
                        new_feedrate = int(command[1:])
                        if new_feedrate != current_feedrate:
                            speed_changes += 1
                            current_feedrate = new_feedrate
                    
                    elif command.startswith("G4"):
                        pause_commands += 1
            
            print(f"Accuracy Analysis Results:")
            print(f"  Total movement distance: {total_distance:.1f}mm")
            print(f"  Total movement commands: {total_moves}")
            print(f"  Average move distance: {total_distance/total_moves:.2f}mm" if total_moves > 0 else "  Average move distance: N/A")
            print(f"  Speed optimizations: {speed_changes}")
            print(f"  Precision pauses: {pause_commands}")
            print(f"  Estimated drawing time: {total_moves * 0.1 + pause_commands * 0.2:.1f}s")
            
            # Accuracy score
            accuracy_score = 100
            if total_moves > 0:
                avg_move = total_distance / total_moves
                if avg_move > GCODE_MAX_SEGMENT_LENGTH:
                    accuracy_score -= 10  # Penalty for large segments
                elif avg_move < GCODE_MIN_SEGMENT_LENGTH:
                    accuracy_score -= 5   # Small penalty for very small segments
            
            if speed_changes < total_moves * 0.1:
                accuracy_score -= 10  # Penalty for not using adaptive speed
            
            if pause_commands < 2:
                accuracy_score -= 5   # Penalty for no precision pauses
            
            print(f"  Accuracy score: {accuracy_score}/100")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to analyze G-code: {e}")
            return False
    
    def export_optimized_gcode(self, contours, filename=None):
        """Export highly optimized G-code with advanced accuracy features."""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"optimized_drawing_{timestamp}.nc"
        
        try:
            # Process contours for optimal accuracy
            processed_contours = []
            total_original_points = 0
            total_optimized_points = 0
            
            print("Optimizing contours for maximum accuracy...")
            
            # Step 0: Optimize contour order for minimal travel
            if OPTIMIZE_DRAWING_PATH and len(contours) > 1:
                ordered_contours = self.optimize_contour_order(contours)
            else:
                ordered_contours = contours
            
            for i, contour_points in enumerate(ordered_contours):
                if len(contour_points) < 2:
                    continue
                
                total_original_points += len(contour_points)
                
                # Step 1: Apply Douglas-Peucker simplification
                simplified = self.douglas_peucker_simplify(contour_points, GCODE_PATH_TOLERANCE)
                
                # Step 2: Apply smoothing if enabled
                smoothed = self.smooth_path(simplified)
                
                # Step 3: Calculate path complexity for adaptive speed
                complexity = self.calculate_path_complexity(smoothed)
                
                processed_contours.append({
                    'points': smoothed,
                    'complexity': complexity,
                    'original_count': len(contour_points),
                    'optimized_count': len(smoothed)
                })
                
                total_optimized_points += len(smoothed)
                
                if i % 10 == 0:  # Progress update every 10 contours
                    print(f"  Processed {i+1}/{len(contours)} contours...")
            
            print(f"✅ Optimization complete:")
            print(f"   Original points: {total_original_points}")
            print(f"   Optimized points: {total_optimized_points}")
            print(f"   Reduction: {((total_original_points - total_optimized_points) / total_original_points * 100):.1f}%")
            
            # Generate G-code
            with open(filename, 'w') as f:
                # Write comprehensive header
                f.write("; Optimized G-code generated by AI Centre Painter\n")
                f.write(f"; Generated: {datetime.now().isoformat()}\n")
                f.write(f"; Total contours: {len(processed_contours)}\n")
                f.write(f"; Total points: {total_optimized_points}\n")
                f.write(f"; Drawing feedrate: {GCODE_DRAWING_FEEDRATE} mm/min\n")
                f.write(f"; Rapid feedrate: {GCODE_RAPID_FEEDRATE} mm/min\n")
                f.write(f"; Path tolerance: {GCODE_PATH_TOLERANCE} mm\n")
                f.write(";\n")
                
                # G-code initialization
                positioning_mode = "G91" if GCODE_USE_RELATIVE_MOVES else "G90"
                f.write(f"{positioning_mode} ; {'Relative' if GCODE_USE_RELATIVE_MOVES else 'Absolute'} positioning\n")
                f.write("G21 ; Units in millimeters\n")
                f.write("G17 ; XY plane selection\n")
                
                if GCODE_PRECISION_MODE:
                    f.write("G61 ; Exact stop mode (precision)\n")
                else:
                    f.write("G64 ; Continuous path mode (smooth)\n")
                
                # Set initial feed rate
                f.write(f"F{GCODE_RAPID_FEEDRATE} ; Set rapid feedrate\n")
                
                # Move to initial position
                f.write(f"G0 X{ORIGIN_X:.{GCODE_DECIMAL_PLACES}f} Y{ORIGIN_Y:.{GCODE_DECIMAL_PLACES}f} Z{SAFE_TRAVEL_HEIGHT:.{GCODE_DECIMAL_PLACES}f}\n")
                f.write(f"G4 P{GCODE_PAUSE_BEFORE_DRAW} ; Pause before starting\n")
                
                # Process each optimized contour
                for contour_data in processed_contours:
                    points = contour_data['points']
                    complexity = contour_data['complexity']
                    
                    if len(points) < 2:
                        continue
                    
                    # Move to start position (pen up)
                    start_x, start_y = points[0]
                    f.write(f"G0 X{start_x:.{GCODE_DECIMAL_PLACES}f} Y{start_y:.{GCODE_DECIMAL_PLACES}f} Z{SAFE_TRAVEL_HEIGHT:.{GCODE_DECIMAL_PLACES}f}\n")
                    
                    # Lower pen to drawing height with controlled speed
                    f.write(f"F{GCODE_APPROACH_FEEDRATE}\n")
                    f.write(f"G1 Z{PEN_DRAWING_Z:.{GCODE_DECIMAL_PLACES}f}\n")
                    f.write(f"G4 P{GCODE_PAUSE_BEFORE_DRAW}\n")
                    
                    # Draw contour segments with adaptive speed
                    current_feedrate = None
                    for point_idx in range(1, len(points)):
                        prev_point = points[point_idx - 1]
                        current_point = points[point_idx]
                        
                        # Calculate optimal feedrate for this segment
                        optimal_feedrate = self.optimize_feedrate_for_segment(
                            prev_point, current_point, complexity
                        )
                        
                        # Only write feedrate if it changed
                        if optimal_feedrate != current_feedrate:
                            f.write(f"F{optimal_feedrate}\n")
                            current_feedrate = optimal_feedrate
                        
                        x, y = current_point
                        f.write(f"G1 X{x:.{GCODE_DECIMAL_PLACES}f} Y{y:.{GCODE_DECIMAL_PLACES}f}\n")
                    
                    # Lift pen after contour
                    f.write(f"G4 P{GCODE_PAUSE_AFTER_DRAW}\n")
                    f.write(f"F{GCODE_RETRACT_FEEDRATE}\n")
                    f.write(f"G1 Z{SAFE_TRAVEL_HEIGHT:.{GCODE_DECIMAL_PLACES}f}\n")
                
                # Final positioning and end
                f.write(f"F{GCODE_RAPID_FEEDRATE}\n")
                f.write(f"G0 X{ORIGIN_X:.{GCODE_DECIMAL_PLACES}f} Y{ORIGIN_Y:.{GCODE_DECIMAL_PLACES}f} Z{SAFE_TRAVEL_HEIGHT:.{GCODE_DECIMAL_PLACES}f}\n")
                f.write("M30 ; Program end and rewind\n")
            
            print(f"✅ Optimized G-code exported to {filename}")
            
            # Validate the generated G-code
            print("\nValidating generated G-code...")
            is_valid, validation_issues = self.validate_gcode_file(filename)
            if is_valid:
                # Analyze accuracy metrics
                self.analyze_gcode_accuracy(filename)
            else:
                print("⚠️  Generated G-code has validation issues. Check settings and try again.")
            
            return filename
            
        except Exception as e:
            print(f"❌ Failed to export optimized G-code: {e}")
            return None

    def export_gcode(self, contours, filename=None):
        """Export contours as G-code (.nc) file - now uses optimized version."""
        return self.export_optimized_gcode(contours, filename)

    def export_inkscape_compatible_gcode(self, contours, filename=None):
        """
        Export G-code in a format more compatible with Inkscape/RoboFlow workflow.
        Uses their coordinate system and formatting conventions.
        """
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"inkscape_drawing_{timestamp}.ngc"
        
        try:
            with open(filename, 'w') as f:
                # Write header comments (Inkscape style)
                f.write("; Generated for ElephantRobotics MyCobot\n")
                f.write(f"; Created: {datetime.now().isoformat()}\n")
                f.write(f"; Drawing area: {INKSCAPE_DRAWING_WIDTH}x{INKSCAPE_DRAWING_HEIGHT}mm\n")
                f.write("; Compatible with RoboFlow trajectory teaching\n")
                f.write(";\n")
                
                # Initialize with absolute positioning
                f.write("G90  ; Absolute positioning\n")
                f.write("G21  ; Units in millimeters\n")
                
                # Home position (will be adjusted by recorded home position)
                f.write("G0 X0 Y0 Z0  ; Move to home position\n")
                f.write("G4 P1000     ; Pause 1 second\n")
                
                for contour_idx, contour_points in enumerate(contours):
                    if len(contour_points) < 2:
                        continue
                    
                    f.write(f"; Contour {contour_idx + 1}\n")
                    
                    # Move to start position (pen up)
                    start_x, start_y = contour_points[0]
                    # Convert to relative coordinates from home position
                    rel_x = start_x - ORIGIN_X
                    rel_y = start_y - ORIGIN_Y
                    f.write(f"G0 X{rel_x:.3f} Y{rel_y:.3f} Z5.0\n")  # 5mm above surface
                    
                    # Lower pen
                    f.write(f"G0 Z0.0\n")  # Touch surface
                    
                    # Draw contour segments
                    for point_idx in range(1, len(contour_points)):
                        x, y = contour_points[point_idx]
                        rel_x = x - ORIGIN_X
                        rel_y = y - ORIGIN_Y
                        f.write(f"G1 X{rel_x:.3f} Y{rel_y:.3f}\n")
                    
                    # Lift pen
                    f.write(f"G0 Z5.0\n")
                
                # Return to home and end
                f.write("; Return to home\n")
                f.write("G0 X0 Y0 Z5.0\n")
                f.write("M30  ; Program end\n")
            
            print(f"✅ Inkscape-compatible NGC exported to {filename}")
            print(f"   Total contours: {len(contours)}")
            print(f"   Compatible with RoboFlow trajectory teaching")
            
            # Validate the generated file
            is_valid, issues = self.validate_ngc_file(filename)
            if not is_valid:
                print(f"⚠️  Generated file has validation issues:")
                for issue in issues[:3]:
                    print(f"    {issue}")
            
            return filename
            
        except Exception as e:
            print(f"❌ Failed to export Inkscape-compatible NGC: {e}")
            return None

    def process_gcode(self, file_path):
        """
        Parse the contents of the gcode file, extract the XYZ coordinate values.
        :param file_path: Gcode file path
        :return: A coordinate list with robot orientation
        """
        # The last valid coordinate, using the current coordinates as starting attitude
        current_coords = self.robot.mc.get_coords()
        if current_coords:
            last_coords = [0.0, 0.0, 0.0, current_coords[3], current_coords[4], current_coords[5]]
        else:
            last_coords = [0.0, 0.0, 0.0] + DRAWING_ORIENTATION
        
        data_coords = []
        
        try:
            with open(file_path, 'r') as file:
                # Line-by-line processing instructions
                for line in file:
                    command = line.strip()  # Remove newline characters and other whitespace characters at the end of the line
                    if command.startswith("G0") or command.startswith("G1"):  # Move command
                        coords = last_coords[:]  # Copy the previous valid coordinates
                        command_parts = command.split()
                        for part in command_parts[1:]:
                            if part.startswith("X") or part.startswith("x"):
                                coords[0] = float(part[1:])  # Extract and transform X coordinate data
                            elif part.startswith("Y") or part.startswith("y"):
                                coords[1] = float(part[1:])  # Extract and transform Y coordinate data
                            elif part.startswith("Z") or part.startswith("z"):
                                coords[2] = float(part[1:])  # Extract and transform Z coordinate data
                        
                        # If XY data is missing, use the last valid XY coordinates
                        if coords[0] == 0.0 and coords[1] == 0.0:
                            coords[0] = last_coords[0]
                            coords[1] = last_coords[1]
                        if coords[2] == 0.0:  # If Z data is missing, use the last valid Z coordinate
                            coords[2] = last_coords[2]
                        
                        last_coords = coords
                        data_coords.append(coords)  # Add coordinates to list and save
        except Exception as e:
            print(f"❌ Failed to parse G-code file: {e}")
            return []
        
        return data_coords

    def process_inkscape_ngc(self, file_path):
        """
        Process NGC files generated by Inkscape with ElephantRobotics plugin.
        These files may have different formatting than our basic G-code.
        """
        print(f"Processing Inkscape NGC file: {file_path}")
        
        # Load home position for proper coordinate reference
        home_pos = self.home_manager.load_home_position()
        if home_pos:
            base_coords = home_pos["cartesian_coords"]
            print(f"Using recorded home position as reference: {base_coords}")
        else:
            print("⚠️  No recorded home position found, using default coordinates")
            base_coords = [ORIGIN_X, ORIGIN_Y, ORIGIN_Z] + DRAWING_ORIENTATION
        
        data_coords = []
        
        try:
            with open(file_path, 'r') as file:
                for line_num, line in enumerate(file, 1):
                    command = line.strip()
                    
                    # Skip empty lines and comments
                    if not command or command.startswith(';') or command.startswith('('):
                        continue
                    
                    # Handle G-code commands
                    if command.startswith("G0") or command.startswith("G1") or command.startswith("G00") or command.startswith("G01"):
                        try:
                            coords = base_coords[:]  # Start with base coordinates
                            command_parts = command.split()
                            
                            for part in command_parts[1:]:
                                if part.startswith("X") or part.startswith("x"):
                                    # Convert from Inkscape coordinate system
                                    x_val = float(part[1:])
                                    coords[0] = base_coords[0] + x_val
                                elif part.startswith("Y") or part.startswith("y"):
                                    # Convert from Inkscape coordinate system  
                                    y_val = float(part[1:])
                                    coords[1] = base_coords[1] + y_val
                                elif part.startswith("Z") or part.startswith("z"):
                                    z_val = float(part[1:])
                                    coords[2] = base_coords[2] + z_val
                            
                            data_coords.append(coords)
                            
                        except ValueError:
                            print(f"⚠️  Line {line_num}: Could not parse coordinates: {command}")
                            continue
                    
                    # Handle other commands (M-codes, etc.)
                    elif command.startswith("M") or command.startswith("G4"):
                        # For now, skip these but could add pause/tool commands later
                        continue
                        
        except Exception as e:
            print(f"❌ Failed to parse Inkscape NGC file: {e}")
            return []
        
        print(f"✅ Parsed {len(data_coords)} movement commands from Inkscape NGC file")
        return data_coords

    def validate_ngc_file(self, file_path):
        """
        Validate an NGC file for RoboFlow compatibility.
        Check for proper formatting and coordinate ranges.
        """
        print(f"Validating NGC file: {file_path}")
        
        issues = []
        line_count = 0
        gcode_commands = 0
        coordinate_ranges = {"X": [], "Y": [], "Z": []}
        
        try:
            with open(file_path, 'r') as file:
                for line_num, line in enumerate(file, 1):
                    line_count += 1
                    command = line.strip()
                    
                    # Skip empty lines and comments
                    if not command or command.startswith(';') or command.startswith('('):
                        continue
                    
                    # Check for G-code movement commands
                    if command.startswith(("G0", "G1", "G00", "G01")):
                        gcode_commands += 1
                        command_parts = command.split()
                        
                        for part in command_parts[1:]:
                            if part.startswith(("X", "x")):
                                try:
                                    x_val = float(part[1:])
                                    coordinate_ranges["X"].append(x_val)
                                except ValueError:
                                    issues.append(f"Line {line_num}: Invalid X coordinate: {part}")
                            elif part.startswith(("Y", "y")):
                                try:
                                    y_val = float(part[1:])
                                    coordinate_ranges["Y"].append(y_val)
                                except ValueError:
                                    issues.append(f"Line {line_num}: Invalid Y coordinate: {part}")
                            elif part.startswith(("Z", "z")):
                                try:
                                    z_val = float(part[1:])
                                    coordinate_ranges["Z"].append(z_val)
                                except ValueError:
                                    issues.append(f"Line {line_num}: Invalid Z coordinate: {part}")
        
        except Exception as e:
            issues.append(f"Failed to read file: {e}")
            return False, issues
        
        # Check coordinate ranges against recommended Inkscape limits
        if coordinate_ranges["X"]:
            x_range = max(coordinate_ranges["X"]) - min(coordinate_ranges["X"])
            if x_range > INKSCAPE_DRAWING_WIDTH:
                issues.append(f"X range ({x_range:.1f}mm) exceeds recommended Inkscape width ({INKSCAPE_DRAWING_WIDTH}mm)")
        
        if coordinate_ranges["Y"]:
            y_range = max(coordinate_ranges["Y"]) - min(coordinate_ranges["Y"])
            if y_range > INKSCAPE_DRAWING_HEIGHT:
                issues.append(f"Y range ({y_range:.1f}mm) exceeds recommended Inkscape height ({INKSCAPE_DRAWING_HEIGHT}mm)")
        
        # Summary
        print(f"Validation Results:")
        print(f"  Lines: {line_count}")
        print(f"  G-code commands: {gcode_commands}")
        
        if coordinate_ranges["X"]:
            print(f"  X range: {min(coordinate_ranges['X']):.1f} to {max(coordinate_ranges['X']):.1f} mm")
        if coordinate_ranges["Y"]:
            print(f"  Y range: {min(coordinate_ranges['Y']):.1f} to {max(coordinate_ranges['Y']):.1f} mm")
        if coordinate_ranges["Z"]:
            print(f"  Z range: {min(coordinate_ranges['Z']):.1f} to {max(coordinate_ranges['Z']):.1f} mm")
        
        if issues:
            print(f"  ⚠️  Issues found: {len(issues)}")
            for issue in issues[:5]:  # Show first 5 issues
                print(f"    - {issue}")
            if len(issues) > 5:
                print(f"    ... and {len(issues) - 5} more issues")
            return False, issues
        else:
            print("  ✅ File validation passed")
            return True, []

    def execute_gcode_drawing(self, file_path, draw_speed=100):
        """
        Execute drawing from a G-code file using MyCobot 320.
        :param file_path: Path to the G-code (.nc) file
        :param draw_speed: Drawing speed (0-100)
        """
        print(f"Processing G-code file: {file_path}")
        
        # Parse G-code file to get coordinate data
        coords_data = self.process_gcode(file_path)
        
        if not coords_data:
            print("❌ No valid coordinates found in G-code file")
            return False
        
        print(f"✅ Loaded {len(coords_data)} coordinate points")
        
        # Move to home position first
        self.robot.go_to_home_position()
        
        # Execute each coordinate command
        print("Starting G-code execution...")
        for i, coords in enumerate(coords_data):
            try:
                # Send coordinates to the robot arm with proper orientation
                self.robot.synchronized_move(coords, draw_speed, 1)  # Use mode 1 for G-code execution
                
                # Progress update every 10 moves
                if i % 10 == 0:
                    progress = (i + 1) / len(coords_data) * 100
                    print(f"Progress: {progress:.1f}% ({i+1}/{len(coords_data)})")
                
            except Exception as e:
                print(f"❌ Error executing coordinate {i+1}: {e}")
                continue
        
        print("✅ G-code execution completed!")
        self.robot.go_to_home_position()
        return True

    def execute_inkscape_ngc(self, file_path, draw_speed=50):
        """
        Execute an Inkscape-generated NGC file using their recommended workflow.
        """
        print(f"\n--- EXECUTING INKSCAPE NGC FILE ---")
        print(f"File: {file_path}")
        
        # Parse the NGC file
        coords_data = self.process_inkscape_ngc(file_path)
        
        if not coords_data:
            print("❌ No valid coordinates found in NGC file")
            return False
        
        print(f"✅ Loaded {len(coords_data)} coordinate points")
        
        # Move to recorded home position first
        if not self.home_manager.go_to_recorded_home():
            print("⚠️  Could not move to recorded home position")
            response = input("Continue with default home position? (y/N): ").lower().strip()
            if response != 'y':
                return False
        
        # Execute trajectory
        print("Starting NGC trajectory execution...")
        start_time = time.time()
        
        for i, coords in enumerate(coords_data):
            try:
                # Send coordinates to robot
                self.robot.synchronized_move(coords, draw_speed, 1)
                
                # Progress update
                if i % 20 == 0:  # Every 20 moves
                    progress = (i + 1) / len(coords_data) * 100
                    elapsed = time.time() - start_time
                    print(f"Progress: {progress:.1f}% ({i+1}/{len(coords_data)}) - {elapsed:.1f}s elapsed")
                
            except Exception as e:
                print(f"❌ Error executing coordinate {i+1}: {e}")
                continue
        
        elapsed_time = time.time() - start_time
        print(f"✅ NGC execution completed in {elapsed_time:.1f}s!")
        
        # Return to home position
        self.home_manager.go_to_recorded_home()
        return True