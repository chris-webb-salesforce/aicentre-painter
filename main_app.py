#!/usr/bin/env python3
"""
AI Centre Painter - Main Application
SIMPLIFIED - Reliable robot drawing system for MyCobot 320.
"""

import cv2
import os
import sys
import time
from datetime import datetime

# Import our modules
from config import *
from robot_controller import RobotController
from image_processor import ImageProcessor
from workspace_manager import WorkspaceManager, HomePositionManager


class DrawingApplication:
    """Simple, reliable drawing application."""

    def __init__(self):
        """Initialize the drawing application."""
        print("\n" + "="*60)
        print("AI Centre Painter - MyCobot 320 Drawing System")
        print("SIMPLIFIED VERSION - Focus on Reliability")
        print("="*60 + "\n")

        try:
            # Initialize core modules
            print("Initializing robot controller...")
            self.robot = RobotController()

            print("Initializing workspace manager...")
            self.workspace = WorkspaceManager(self.robot)
            self.home_manager = HomePositionManager(self.robot)

            print("Initializing image processor...")
            self.image_processor = ImageProcessor()

            print("✅ All modules initialized successfully!")

        except Exception as e:
            print(f"❌ Failed to initialize application: {e}")
            sys.exit(1)

    def draw_sketch(self, sketch_image):
        """
        Draw the sketch - SIMPLE AND RELIABLE.
        This is the core function.
        """
        if sketch_image is None:
            print("Cannot draw, sketch image is missing.")
            return

        # Preprocess contours
        print("\nPreprocessing contours...")
        contours = self.image_processor.preprocess_contours(sketch_image)

        if not contours:
            print("No contours found in sketch.")
            return

        # Validate trajectory points
        if not self.workspace.validate_trajectory_points(contours):
            print("❌ Trajectory validation failed!")
            response = input("Continue anyway? (y/N): ").lower().strip()
            if response != 'y':
                return

        # Create trajectory summary
        self.workspace.create_trajectory_summary(contours)

        # Optimize path order if enabled
        if OPTIMIZE_DRAWING_PATH:
            print("Optimizing path order...")
            contours = self.image_processor.optimize_contour_path(contours)

        # Create and show preview
        print("Creating drawing preview...")
        preview_img = self.image_processor.create_contour_preview(contours)
        cv2.imshow("Drawing Preview", preview_img)

        print("\nPreview Controls:")
        print("  'd' = Start drawing")
        print("  's' = Save preview image")
        print("  Any other key = Cancel")

        key = cv2.waitKey(0)
        cv2.destroyAllWindows()

        if key == ord('s'):
            preview_filename = f"preview_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(preview_filename, preview_img)
            print(f"Preview saved as {preview_filename}")
            print("Press 'd' to draw or any other key to cancel:")
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()

        if key != ord('d'):
            print("Drawing cancelled.")
            return

        # === START DRAWING ===
        print("\n" + "="*60)
        print(f"STARTING DRAWING - {len(contours)} contours")
        print("="*60)

        # Go to home position
        print("\nMoving to home position...")
        if not self.robot.go_to_home_position():
            print("❌ Failed to reach home position")
            return
        time.sleep(1)

        start_time = time.time()
        successful_contours = 0
        failed_contours = 0

        # Draw each contour
        for i, contour_points in enumerate(contours):
            if len(contour_points) < 2:
                print(f"Skipping contour {i+1} (too few points)")
                continue

            print(f"\n--- Contour {i+1}/{len(contours)} ({len(contour_points)} points) ---")

            try:
                # Move to start position (pen up)
                start_x, start_y, start_z = contour_points[0]
                print(f"Moving to start: ({start_x:.1f}, {start_y:.1f})")

                if not self.robot.move_to(start_x, start_y, SAFE_TRAVEL_HEIGHT):
                    print(f"⚠️  Failed to reach start position")
                    failed_contours += 1
                    continue

                # Lower pen
                if not self.robot.pen_down():
                    print(f"⚠️  Failed to lower pen")
                    failed_contours += 1
                    continue

                # Draw all points in this contour
                points_drawn = 0
                for j in range(1, len(contour_points)):
                    next_x, next_y, next_z = contour_points[j]

                    if self.robot.draw_to(next_x, next_y):
                        points_drawn += 1
                    else:
                        print(f"⚠️  Failed to draw point {j}")

                    # Progress indicator every 10 points
                    if j % 10 == 0:
                        print(f"  Progress: {j}/{len(contour_points)} points")

                print(f"  Drew {points_drawn}/{len(contour_points)-1} segments")

                # Lift pen
                if not self.robot.pen_up():
                    print(f"⚠️  Failed to lift pen")

                successful_contours += 1

            except Exception as e:
                print(f"❌ Error drawing contour {i+1}: {e}")
                failed_contours += 1
                # Try to lift pen if something failed
                try:
                    self.robot.pen_up()
                except:
                    pass

        # === DRAWING COMPLETE ===
        elapsed_time = time.time() - start_time

        print("\n" + "="*60)
        print("DRAWING COMPLETE")
        print("="*60)
        print(f"Successful: {successful_contours}/{len(contours)} contours")
        print(f"Failed: {failed_contours}/{len(contours)} contours")
        print(f"Time: {elapsed_time//60:.0f}m {elapsed_time%60:.0f}s")
        print(f"Average: {elapsed_time/len(contours):.1f}s per contour")
        print("="*60)

        # Return to home
        print("\nReturning to home position...")
        self.robot.go_to_home_position()

    def create_new_drawing(self):
        """Create a new drawing from camera or test image."""
        # For now, use existing test image or prompt for image
        if os.path.exists(CAPTURED_IMAGE_PATH):
            use_existing = input(f"Use existing image '{CAPTURED_IMAGE_PATH}'? (Y/n): ").lower().strip()
            if use_existing != 'n':
                print(f"Using existing image: {CAPTURED_IMAGE_PATH}")
            else:
                # Capture new image
                if not self.image_processor.capture_image():
                    print("Image capture cancelled.")
                    return
        else:
            # Capture new image
            if not self.image_processor.capture_image():
                print("Image capture cancelled.")
                return

        # Create sketch
        print("\nCreating sketch...")
        sketch = self.image_processor.create_sketch()

        if sketch is not None:
            cv2.imshow("Generated Sketch - Press 'd' to draw, any key to cancel", sketch)
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()

            if key == ord('d'):
                self.draw_sketch(sketch)
            else:
                print("Drawing cancelled.")
        else:
            print("Failed to create sketch.")

    def run_main_loop(self):
        """Run the main application loop."""
        while True:
            print("\n" + "="*60)
            print("MAIN MENU")
            print("="*60)
            print("1. Create new drawing from image")
            print("2. Test drawing area (draw rectangle)")
            print("3. Calibrate pen pressure")
            print("4. Test coordinate system")
            print("5. Exit")
            print("="*60)

            action = input("Select option (1-5): ").strip()

            if action == '5' or action.lower() == 'exit':
                break
            elif action == '1':
                self.create_new_drawing()
            elif action == '2':
                self.workspace.test_drawing_area()
            elif action == '3':
                self.home_manager.calibrate_pen_pressure()
            elif action == '4':
                self.workspace.test_coordinate_system()
            else:
                print("Invalid option. Please select 1-5.")

    def run(self):
        """Main entry point for the application."""
        try:
            # Check for required files
            if not os.path.exists(HAAR_CASCADE_PATH):
                print(f"\n⚠️  WARNING: Haar Cascade file not found.")
                print(f"Path: {HAAR_CASCADE_PATH}")
                print("Face detection will be disabled.")

            # Run main application loop
            self.run_main_loop()

            print("\nShutting down gracefully...")

        except KeyboardInterrupt:
            print("\n\nInterrupted by user. Shutting down...")
        except Exception as e:
            print(f"\n❌ Application error: {e}")
        finally:
            # Ensure robot is in safe position
            try:
                print("Moving to safe position...")
                self.robot.go_to_safe_position()
            except:
                pass


def main():
    """Main entry point."""
    app = DrawingApplication()
    app.run()


if __name__ == "__main__":
    main()
