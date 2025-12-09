#!/usr/bin/env python3
"""
AI Centre Painter - MyCobot 320 Drawing System
"""

import cv2
import os
import sys
import time
from datetime import datetime

from config import *
from robot_controller import RobotController
from image_processor import ImageProcessor
from workspace_manager import WorkspaceManager

try:
    from calibration_wizard import CalibrationWizard
    WIZARD_AVAILABLE = True
except ImportError:
    WIZARD_AVAILABLE = False


class DrawingApplication:

    def __init__(self):
        print("\n" + "="*60)
        print("AI Centre Painter - MyCobot 320 Drawing System")
        print("="*60 + "\n")

        try:
            print("Initializing robot controller...")
            self.robot = RobotController()

            print("Initializing workspace manager...")
            self.workspace = WorkspaceManager(self.robot)

            print("Initializing image processor...")
            self.image_processor = ImageProcessor()

            print("All modules initialized successfully!")

        except Exception as e:
            print(f"Failed to initialize application: {e}")
            sys.exit(1)

    def draw_sketch(self, sketch_image):
        """Draw the sketch."""
        if sketch_image is None:
            print("Cannot draw, sketch image is missing.")
            return

        print("\nPreprocessing contours...")
        contours = self.image_processor.preprocess_contours(sketch_image)

        if not contours:
            print("No contours found in sketch.")
            return

        if not self.workspace.validate_trajectory_points(contours):
            print("Trajectory validation warning!")
            response = input("Continue anyway? (y/N): ").lower().strip()
            if response != 'y':
                return

        if OPTIMIZE_DRAWING_PATH:
            print("Optimizing path order...")
            contours = self.image_processor.optimize_contour_path(contours)

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

        print("\n" + "="*60)
        print(f"STARTING DRAWING - {len(contours)} contours")
        print("="*60)

        self.robot.go_to_home_position()
        time.sleep(1)

        try:
            self.robot.draw_full_sketch(contours)
        except KeyboardInterrupt:
            print("\nSTOPPING DRAWING")
            self.robot.pen_up()
            self.robot.mc.stop()

        print("\n" + "="*60)
        print("DRAWING COMPLETE")
        print("="*60)

    def run_calibration_wizard(self):
        """Run the interactive surface calibration."""
        if not WIZARD_AVAILABLE:
            print("Calibration Wizard module not found.")
            return

        print("\nLaunching Calibration Wizard...")
        try:
            wizard = CalibrationWizard()
            wizard.run()

            print("Reloading calibration data...")
            self.robot.surface.load_calibration()

        except Exception as e:
            print(f"Error running wizard: {e}")

    def create_new_drawing(self):
        """Create a new drawing from camera or test image."""
        if os.path.exists(CAPTURED_IMAGE_PATH):
            use_existing = input(f"Use existing image '{CAPTURED_IMAGE_PATH}'? (Y/n): ").lower().strip()
            if use_existing != 'n':
                print(f"Using existing image: {CAPTURED_IMAGE_PATH}")
            else:
                if not self.image_processor.capture_image():
                    return
        else:
            if not self.image_processor.capture_image():
                return

        print("\nCreating sketch...")
        sketch = self.image_processor.create_sketch()

        if sketch is not None:
            cv2.imshow("Generated Sketch", sketch)
            print("Press 'd' to draw, any other key to cancel")
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
            print("2. Run Surface Calibration Wizard (3-Point)")
            print("3. Test drawing area (Rectangle)")
            print("4. Exit")
            print("="*60)

            action = input("Select option (1-4): ").strip()

            if action == '4' or action.lower() == 'exit':
                break
            elif action == '1':
                self.create_new_drawing()
            elif action == '2':
                self.run_calibration_wizard()
            elif action == '3':
                print("Drawing test rectangle...")
                box = [
                    (ORIGIN_X + 20, ORIGIN_Y + 20),
                    (ORIGIN_X + 80, ORIGIN_Y + 20),
                    (ORIGIN_X + 80, ORIGIN_Y + 80),
                    (ORIGIN_X + 20, ORIGIN_Y + 80),
                    (ORIGIN_X + 20, ORIGIN_Y + 20)
                ]
                self.robot.draw_full_sketch([box])
            else:
                print("Invalid option.")

    def run(self):
        """Main entry point."""
        try:
            if not os.path.exists(HAAR_CASCADE_PATH):
                print("\nWARNING: Haar Cascade file not found.")

            self.run_main_loop()
            print("\nShutting down...")

        except KeyboardInterrupt:
            print("\n\nInterrupted by user. Shutting down...")
        except Exception as e:
            print(f"\nApplication error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            try:
                print("Moving to safe position...")
                self.robot.go_to_home_position()
            except:
                pass

def main():
    app = DrawingApplication()
    app.run()

if __name__ == "__main__":
    main()
