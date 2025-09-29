#!/usr/bin/env python3
"""
AI Centre Painter - Main Application Controller
Refactored modular robot drawing system for MyCobot 320.
"""

import cv2
import os
import sys
import time
import numpy as np
from datetime import datetime

# Import our modules
from config import *
from robot_controller import RobotController
from gcode_handler import GCodeHandler
from image_processor import ImageProcessor
from workspace_manager import WorkspaceManager, HomePositionManager


class DrawingApplication:
    """Main application controller that coordinates all modules."""
    
    def __init__(self):
        """Initialize the drawing application."""
        print("\n--- AI Centre Painter - MyCobot 320 Drawing System ---")
        print("Modular robot drawing system with ElephantRobotics workflow support\n")
        
        try:
            # Initialize core modules
            print("Initializing robot controller...")
            self.robot = RobotController()
            
            print("Initializing workspace manager...")
            self.workspace = WorkspaceManager(self.robot)
            self.home_manager = HomePositionManager(self.robot)
            
            print("Initializing G-code handler...")
            self.gcode = GCodeHandler(self.robot, self.home_manager)
            
            print("Initializing image processor...")
            self.image_processor = ImageProcessor()
            
            print("✅ All modules initialized successfully!")
            
        except Exception as e:
            print(f"❌ Failed to initialize application: {e}")
            sys.exit(1)

    def print_progress_bar(self, current, total, bar_length=50, prefix="Progress"):
        """Print a progress bar to the terminal."""
        if total == 0:
            return
        
        progress = current / total
        filled_length = int(bar_length * progress)
        bar = '█' * filled_length + '░' * (bar_length - filled_length)
        percent = progress * 100
        
        # Print with carriage return to update same line
        print(f'\r{prefix}: |{bar}| {percent:.1f}% ({current}/{total})', end='', flush=True)
        
        # Print newline when complete
        if current >= total:
            print()

    def handle_trajectory_export(self, contours):
        """Handle interactive trajectory export."""
        print("\n--- TRAJECTORY EXPORT ---")
        print("Choose export format:")
        print("  1) JSON (recommended for PyBullet)")
        print("  2) CSV (for spreadsheet analysis)")
        print("  3) G-code (.nc) for MyCobot execution")
        print("  4) Inkscape-compatible NGC for RoboFlow")
        print("  5) All formats")
        
        choice = input("Enter choice (1-5): ").strip()
        
        exported_files = []
        
        if choice in ['1', '5']:
            # Export JSON (placeholder - would need PyBullet export from original code)
            print("JSON export not implemented in refactor")
        
        if choice in ['2', '5']:
            # Export CSV (placeholder - would need CSV export from original code)
            print("CSV export not implemented in refactor")
        
        if choice in ['3', '5']:
            # Export G-code
            gcode_file = self.gcode.export_gcode(contours)
            if gcode_file:
                exported_files.append(gcode_file)
        
        if choice in ['4', '5']:
            # Export Inkscape-compatible NGC
            ngc_file = self.gcode.export_inkscape_compatible_gcode(contours)
            if ngc_file:
                exported_files.append(ngc_file)
        
        if exported_files:
            print(f"\n✅ Export complete! Files created:")
            for file in exported_files:
                print(f"   • {file}")
        else:
            print("No files exported.")

    def draw_sketch_direct(self, sketch_image):
        """Draw the sketch directly using robot control."""
        if sketch_image is None:
            print("Cannot draw, sketch image is missing.")
            return
        
        # Preprocess all contours at once
        print("Preprocessing contours...")
        contours = self.image_processor.preprocess_contours(sketch_image)
        
        # Validate trajectory points
        if not self.workspace.validate_trajectory_points(contours):
            print("❌ Trajectory validation failed! Please check workspace settings.")
            response = input("Continue anyway? (y/N): ").lower().strip()
            if response != 'y':
                return
        
        # Create trajectory summary
        self.workspace.create_trajectory_summary(contours)
        
        if OPTIMIZE_DRAWING_PATH:
            print("Optimizing path...")
            contours = self.image_processor.optimize_contour_path(contours)
        
        # Create and show preview
        print("Creating drawing preview...")
        preview_img = self.image_processor.create_contour_preview(contours)
        cv2.imshow("Drawing Preview - Check path and order", preview_img)
        
        print("\nPreview Controls:")
        print("  'd' = Start drawing")
        print("  's' = Save preview image")
        print("  'e' = Export trajectory for PyBullet simulation")
        print("  'g' = Export G-code (.nc) file for MyCobot execution")
        print("  'i' = Export Inkscape-compatible NGC file for RoboFlow")
        print("  Any other key = Cancel drawing")
        
        key = cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        if key == ord('s'):
            preview_filename = "drawing_preview.jpg"
            cv2.imwrite(preview_filename, preview_img)
            print(f"Preview saved as {preview_filename}")
            print("Press 'd' to draw, 'e' to export, or any other key to cancel:")
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
        elif key == ord('e'):
            self.handle_trajectory_export(contours)
            print("Press 'd' to draw, 'g' to export G-code, or any other key to cancel:")
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
        elif key == ord('g'):
            # Export G-code
            gcode_file = self.gcode.export_gcode(contours)
            if gcode_file:
                print(f"G-code exported to {gcode_file}")
                execute_now = input("Execute G-code drawing now? (y/n): ").lower().strip()
                if execute_now == 'y':
                    speed = input("Enter drawing speed (1-100, default 50): ").strip()
                    try:
                        draw_speed = int(speed) if speed else 50
                        draw_speed = max(1, min(100, draw_speed))
                    except ValueError:
                        draw_speed = 50
                    self.gcode.execute_gcode_drawing(gcode_file, draw_speed)
            print("Press 'd' to draw directly or any other key to cancel:")
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
        elif key == ord('i'):
            # Export Inkscape-compatible NGC
            ngc_file = self.gcode.export_inkscape_compatible_gcode(contours)
            if ngc_file:
                print(f"Inkscape-compatible NGC exported to {ngc_file}")
                execute_now = input("Execute NGC drawing now? (y/n): ").lower().strip()
                if execute_now == 'y':
                    speed = input("Enter drawing speed (1-100, default 50): ").strip()
                    try:
                        draw_speed = int(speed) if speed else 50
                        draw_speed = max(1, min(100, draw_speed))
                    except ValueError:
                        draw_speed = 50
                    self.gcode.execute_inkscape_ngc(ngc_file, draw_speed)
            print("Press 'd' to draw directly or any other key to cancel:")
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        if key != ord('d'):
            print("Drawing cancelled.")
            return
        
        print(f"Drawing {len(contours)} optimized contours...")
        print("="*60)
        
        # Setup
        self.robot.go_to_home_position()
        time.sleep(1)
        
        start_time = time.time()
        
        # Draw all contours with progress tracking
        for i, contour_points in enumerate(contours):
            if len(contour_points) < 2:
                continue
            
            # Progress update every 50 contours
            if i % 50 == 0:
                self.print_progress_bar(i+1, len(contours), prefix="Drawing")
            
            # Start drawing this contour
            start_x, start_y = contour_points[0]
            self.robot.gentle_pen_down(start_x, start_y)
            
            # Draw all segments in this contour
            for j in range(1, len(contour_points)):
                prev_x, prev_y = contour_points[j-1]
                next_x, next_y = contour_points[j]
                
                # Use proper line segment drawing
                self.robot.draw_line_segment((prev_x, prev_y), (next_x, next_y))
            
            # Lift pen for next contour
            self.robot.gentle_pen_up()
        
        # Final progress and timing
        elapsed_time = time.time() - start_time
        print()
        print("="*60)
        print(f"✓ Drawing complete!")
        print(f"  Contours drawn: {len(contours)}")
        print(f"  Time elapsed: {elapsed_time//60:.0f}m {elapsed_time%60:.0f}s")
        print(f"  Average: {elapsed_time/len(contours):.3f}s per contour")
        print("="*60)
        self.robot.go_to_home_position()

    def create_new_drawing(self):
        """Create a new drawing from camera image."""
        # Photo capture
        # Skip for now - can be uncommented when needed
        # self.robot.go_to_photo_position()
        # if not self.image_processor.capture_image():
        #     print("Image capture cancelled.")
        #     self.robot.go_to_home_position()
        #     return
        
        # Create sketch
        sketch = self.image_processor.create_sketch()
        if sketch is not None:
            cv2.imshow("Generated Sketch - Press 'd' to draw, any key to cancel", sketch)
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            if key == ord('d'):
                self.draw_sketch_direct(sketch)
            else:
                print("Drawing cancelled.")
                self.robot.go_to_home_position()
        else:
            print("Failed to create sketch.")

    def run_initial_setup(self):
        """Run initial setup and calibration options."""
        # Movement synchronization option
        sync_option = input("Use movement synchronization? (Y/n): ").lower().strip()
        if sync_option == 'n':
            self.robot.use_movement_sync = False
            print("Movement synchronization disabled - using time-based delays")
        else:
            print("Movement synchronization enabled - waiting for completion")
        
        # Offer calibration option
        calibrate = input("Would you like to calibrate pen pressure? (y/n): ").lower().strip()
        if calibrate == 'y':
            self.home_manager.calibrate_pen_pressure()
        
        # Test coordinate system
        coord_test = input("Would you like to test the coordinate system? (y/n): ").lower().strip()
        if coord_test == 'y':
            self.workspace.test_coordinate_system()
        
        # Test drawing area
        test = input("Would you like to test the drawing area? (y/n): ").lower().strip()
        if test == 'y':
            self.workspace.test_drawing_area()

    def run_main_loop(self):
        """Run the main application loop."""
        while True:
            print("\n--- MAIN MENU ---")
            print("1. Create new drawing from image")
            print("2. Execute existing G-code file")
            print("3. Execute Inkscape NGC file (recommended workflow)")
            print("4. Record home position (pen touching drawing board)")
            print("5. Test recorded home position")
            print("6. Exit")
            
            action = input("Select option (1-6): ").strip()
            
            if action == '6' or action.lower() == 'exit':
                break
            elif action == '2':
                # Execute existing G-code file
                gcode_file = input("Enter G-code file path: ").strip()
                if os.path.exists(gcode_file):
                    speed = input("Enter drawing speed (1-100, default 50): ").strip()
                    try:
                        draw_speed = int(speed) if speed else 50
                        draw_speed = max(1, min(100, draw_speed))
                    except ValueError:
                        draw_speed = 50
                    self.gcode.execute_gcode_drawing(gcode_file, draw_speed)
                else:
                    print(f"File not found: {gcode_file}")
                continue
            elif action == '3':
                # Execute Inkscape NGC file
                ngc_file = input("Enter Inkscape NGC file path: ").strip()
                if os.path.exists(ngc_file):
                    speed = input("Enter drawing speed (1-100, default 50): ").strip()
                    try:
                        draw_speed = int(speed) if speed else 50
                        draw_speed = max(1, min(100, draw_speed))
                    except ValueError:
                        draw_speed = 50
                    self.gcode.execute_inkscape_ngc(ngc_file, draw_speed)
                else:
                    print(f"File not found: {ngc_file}")
                continue
            elif action == '4':
                # Record home position
                print("\n--- HOME POSITION RECORDING ---")
                print("Follow these steps from the documentation:")
                print("1. Use RoboFlow's quickmove to position the robot arm")
                print("2. Control the arm to lightly press the pen tip on the drawing board")
                print("3. Press Enter below to record this position")
                self.home_manager.record_home_position()
                continue
            elif action == '5':
                # Test recorded home position
                print("\n--- TESTING RECORDED HOME POSITION ---")
                if self.home_manager.go_to_recorded_home():
                    print("✅ Successfully moved to recorded home position")
                else:
                    print("❌ Failed to move to recorded home position")
                continue
            elif action != '1':
                print("Invalid option. Please select 1-6.")
                continue
            
            # Option 1: Create new drawing
            self.create_new_drawing()

    def run(self):
        """Main entry point for the application."""
        try:
            # Check for required files
            if not os.path.exists(HAAR_CASCADE_PATH):
                print(f"\n--- WARNING ---")
                print("Haar Cascade file not found.")
                print(f"Path: {HAAR_CASCADE_PATH}")
                print("Face detection will be disabled.")
            
            # Run initial setup
            self.run_initial_setup()
            
            # Run main application loop
            self.run_main_loop()
            
            print("Shutting down gracefully.")
            
        except KeyboardInterrupt:
            print("\n\nInterrupted by user. Shutting down...")
        except Exception as e:
            print(f"\n❌ Application error: {e}")
        finally:
            # Ensure robot is in safe position
            try:
                self.robot.go_to_safe_position()
            except:
                pass


def main():
    """Main entry point."""
    app = DrawingApplication()
    app.run()


if __name__ == "__main__":
    main()