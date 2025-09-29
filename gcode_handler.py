"""
G-code and NGC file handling module.
Handles parsing, validation, and execution of G-code files.
"""

import os
import time
from datetime import datetime
from config import *


class GCodeHandler:
    """Handle G-code and NGC file operations."""
    
    def __init__(self, robot_controller, home_position_manager):
        """Initialize with robot controller and home position manager."""
        self.robot = robot_controller
        self.home_manager = home_position_manager
    
    def export_gcode(self, contours, filename=None):
        """Export contours as G-code (.nc) file compatible with MyCobot workflow."""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"drawing_{timestamp}.nc"
        
        try:
            with open(filename, 'w') as f:
                # Write G-code header
                f.write("G90\n")  # Absolute positioning
                f.write("G4 S1\n")  # Pause 1 second
                
                # Initial positioning - move to safe height
                f.write(f"G01 X{ORIGIN_X:.2f} Y{ORIGIN_Y:.2f} Z{PEN_RETRACT_Z:.2f} F15\n")
                f.write("G4 S1\n")  # Pause
                
                for contour_points in contours:
                    if len(contour_points) < 2:
                        continue
                    
                    # Move to start position (pen up)
                    start_x, start_y = contour_points[0]
                    f.write(f"G0 X{start_x:.2f} Y{start_y:.2f}\n")
                    
                    # Lower pen to drawing height
                    f.write(f"G0 Z{PEN_DRAWING_Z:.2f}\n")
                    
                    # Draw contour segments
                    for point_idx in range(1, len(contour_points)):
                        x, y = contour_points[point_idx]
                        f.write(f"G1 X{x:.2f} Y{y:.2f}\n")
                    
                    # Lift pen after contour
                    f.write(f"G0 Z{PEN_RETRACT_Z:.2f}\n")
                
                # Final position and end
                f.write(f"G0 X{ORIGIN_X:.2f} Y{ORIGIN_Y:.2f}\n")
                f.write("M21 P0\n")  # Program end
            
            print(f"✅ G-code exported to {filename}")
            print(f"   Total contours: {len(contours)}")
            return filename
            
        except Exception as e:
            print(f"❌ Failed to export G-code: {e}")
            return None

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