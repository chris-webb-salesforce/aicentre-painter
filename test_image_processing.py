#!/usr/bin/env python3
"""
Standalone Image Processing and Path Testing Tool
Test image processing and draw path calculation without robot connection.
"""

import cv2
import numpy as np
import os
import sys
from datetime import datetime

# Import our modules
from config import *
from image_processor import ImageProcessor


class ImageProcessingTester:
    """Test image processing and path generation in isolation."""

    def __init__(self):
        """Initialize the tester."""
        print("\n" + "="*60)
        print("Image Processing & Path Testing Tool")
        print("Test drawing paths without robot connection")
        print("="*60 + "\n")

        self.image_processor = ImageProcessor()
        self.current_image = None
        self.current_sketch = None
        self.current_contours = None

    def load_image(self, image_path):
        """Load an image from file."""
        if not os.path.exists(image_path):
            print(f"❌ Image not found: {image_path}")
            return False

        self.current_image = cv2.imread(image_path)
        if self.current_image is None:
            print(f"❌ Failed to load image: {image_path}")
            return False

        print(f"✅ Loaded image: {image_path}")
        print(f"   Size: {self.current_image.shape[1]}x{self.current_image.shape[0]}")
        return True

    def load_sketch(self, sketch_path):
        """Load a sketch directly (bypassing image-to-sketch conversion)."""
        if not os.path.exists(sketch_path):
            print(f"❌ Sketch not found: {sketch_path}")
            return False

        self.current_sketch = cv2.imread(sketch_path, cv2.IMREAD_GRAYSCALE)
        if self.current_sketch is None:
            print(f"❌ Failed to load sketch: {sketch_path}")
            return False

        print(f"✅ Loaded sketch: {sketch_path}")
        print(f"   Size: {self.current_sketch.shape[1]}x{self.current_sketch.shape[0]}")
        return True

    def capture_new_image(self):
        """Capture a new image from camera."""
        print("\nCapturing image from camera...")
        if self.image_processor.capture_image():
            return self.load_image(CAPTURED_IMAGE_PATH)
        return False

    def generate_sketch(self):
        """Generate sketch from current image."""
        if self.current_image is None:
            print("❌ No image loaded. Load an image first.")
            return False

        print("\nGenerating sketch...")
        self.current_sketch = self.image_processor.create_sketch()

        if self.current_sketch is None:
            print("❌ Failed to generate sketch")
            return False

        print("✅ Sketch generated successfully")
        return True

    def process_contours(self):
        """Process contours from sketch."""
        if self.current_sketch is None:
            print("❌ No sketch available. Generate a sketch first.")
            return False

        print("\nProcessing contours...")
        self.current_contours = self.image_processor.preprocess_contours(self.current_sketch)

        if not self.current_contours:
            print("❌ No contours found in sketch")
            return False

        print(f"✅ Found {len(self.current_contours)} contours")
        return True

    def optimize_path(self):
        """Optimize contour drawing order."""
        if not self.current_contours:
            print("❌ No contours to optimize. Process contours first.")
            return False

        print("\nOptimizing drawing path...")
        original_count = len(self.current_contours)
        self.current_contours = self.image_processor.optimize_contour_path(self.current_contours)
        print(f"✅ Optimized {original_count} contours")
        return True

    def show_image(self):
        """Display the current source image."""
        if self.current_image is None:
            print("❌ No image loaded")
            return

        cv2.imshow("Source Image - Press any key to close", self.current_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def show_sketch(self):
        """Display the generated sketch."""
        if self.current_sketch is None:
            print("❌ No sketch generated")
            return

        cv2.imshow("Generated Sketch - Press any key to close", self.current_sketch)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def show_preview(self, save_on_exit=False):
        """Show contour preview with drawing order."""
        if not self.current_contours:
            print("❌ No contours to preview")
            return

        print("\nGenerating preview...")
        preview_img = self.image_processor.create_contour_preview(self.current_contours)

        print("\nPreview Controls:")
        print("  's' = Save preview and close")
        print("  Any other key = Close without saving")

        cv2.imshow("Drawing Preview", preview_img)
        key = cv2.waitKey(0)
        cv2.destroyAllWindows()

        if key == ord('s'):
            preview_filename = f"preview_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(preview_filename, preview_img)
            print(f"✅ Preview saved as {preview_filename}")

    def show_statistics(self):
        """Display detailed statistics about the current path."""
        if not self.current_contours:
            print("❌ No contours to analyze")
            return

        print("\n" + "="*60)
        print("PATH STATISTICS")
        print("="*60)

        total_points = sum(len(contour) for contour in self.current_contours)
        print(f"Total contours: {len(self.current_contours)}")
        print(f"Total points: {total_points}")
        print(f"Average points per contour: {total_points / len(self.current_contours):.1f}")

        # Calculate drawing distance
        drawing_distance = 0
        travel_distance = 0

        for i, contour in enumerate(self.current_contours):
            # Distance within contour (drawing)
            for j in range(1, len(contour)):
                p1 = contour[j-1]
                p2 = contour[j]
                dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                drawing_distance += dist

            # Distance to next contour (travel)
            if i < len(self.current_contours) - 1:
                end_point = contour[-1]
                next_start = self.current_contours[i+1][0]
                dist = np.sqrt((next_start[0] - end_point[0])**2 + (next_start[1] - end_point[1])**2)
                travel_distance += dist

        print(f"\nDistance:")
        print(f"  Drawing: {drawing_distance:.1f} mm")
        print(f"  Travel: {travel_distance:.1f} mm")
        print(f"  Total: {drawing_distance + travel_distance:.1f} mm")

        # Estimate time (rough calculation)
        drawing_time = drawing_distance / DRAWING_SPEED
        travel_time = travel_distance / TRAVEL_SPEED
        # Add time for pen lifts/lowers
        pen_operations = len(self.current_contours) * 2  # up and down per contour
        pen_time = pen_operations * 0.5  # Rough estimate

        total_time = drawing_time + travel_time + pen_time

        print(f"\nEstimated time:")
        print(f"  Drawing: {drawing_time:.1f}s")
        print(f"  Travel: {travel_time:.1f}s")
        print(f"  Pen operations: {pen_time:.1f}s")
        print(f"  Total: {total_time:.1f}s ({total_time/60:.1f} minutes)")

        # Workspace bounds
        all_x = [p[0] for contour in self.current_contours for p in contour]
        all_y = [p[1] for contour in self.current_contours for p in contour]

        print(f"\nWorkspace bounds:")
        print(f"  X: {min(all_x):.1f} to {max(all_x):.1f} mm (range: {max(all_x) - min(all_x):.1f} mm)")
        print(f"  Y: {min(all_y):.1f} to {max(all_y):.1f} mm (range: {max(all_y) - min(all_y):.1f} mm)")

        # Check if within bounds
        expected_x_range = (ORIGIN_X, ORIGIN_X + DRAWING_AREA_WIDTH_MM)
        expected_y_range = (ORIGIN_Y, ORIGIN_Y + DRAWING_AREA_HEIGHT_MM)

        x_ok = min(all_x) >= expected_x_range[0] and max(all_x) <= expected_x_range[1]
        y_ok = min(all_y) >= expected_y_range[0] and max(all_y) <= expected_y_range[1]

        print(f"\nBounds check:")
        print(f"  X: {'✅ OK' if x_ok else '❌ OUT OF BOUNDS'}")
        print(f"  Y: {'✅ OK' if y_ok else '❌ OUT OF BOUNDS'}")

        # Contour size distribution
        sizes = [len(c) for c in self.current_contours]
        print(f"\nContour size distribution:")
        print(f"  Smallest: {min(sizes)} points")
        print(f"  Largest: {max(sizes)} points")
        print(f"  Median: {sorted(sizes)[len(sizes)//2]} points")

        print("="*60)

    def save_sketch(self, filename=None):
        """Save the current sketch to file."""
        if self.current_sketch is None:
            print("❌ No sketch to save")
            return

        if filename is None:
            filename = f"sketch_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"

        cv2.imwrite(filename, self.current_sketch)
        print(f"✅ Sketch saved as {filename}")

    def full_pipeline(self, image_path, is_sketch=False):
        """Run full processing pipeline on an image or sketch."""
        print("\n" + "="*60)
        print("RUNNING FULL PROCESSING PIPELINE")
        print("="*60)

        if is_sketch:
            # Load sketch directly
            if not self.load_sketch(image_path):
                return False
        else:
            # Load image and generate sketch
            if not self.load_image(image_path):
                return False

            # Generate sketch
            if not self.generate_sketch():
                return False

        # Process contours
        if not self.process_contours():
            return False

        # Optimize path
        if OPTIMIZE_DRAWING_PATH:
            self.optimize_path()

        # Show statistics
        self.show_statistics()

        # Show preview
        self.show_preview()

        return True

    def interactive_menu(self):
        """Interactive menu for testing."""
        while True:
            print("\n" + "="*60)
            print("IMAGE PROCESSING TEST MENU")
            print("="*60)
            print("1. Load image from file")
            print("2. Load sketch from file (skip conversion)")
            print("3. Capture image from camera")
            print("4. Generate sketch from image")
            print("5. Process contours")
            print("6. Optimize drawing path")
            print("7. Show source image")
            print("8. Show sketch")
            print("9. Show drawing preview")
            print("10. Show statistics")
            print("11. Save sketch")
            print("12. Run full pipeline on image")
            print("13. Run full pipeline on sketch")
            print("14. Exit")
            print("="*60)

            choice = input("Select option (1-14): ").strip()

            if choice == '1':
                path = input("Enter image path: ").strip()
                self.load_image(path)
            elif choice == '2':
                path = input("Enter sketch path: ").strip()
                self.load_sketch(path)
            elif choice == '3':
                self.capture_new_image()
            elif choice == '4':
                self.generate_sketch()
            elif choice == '5':
                self.process_contours()
            elif choice == '6':
                self.optimize_path()
            elif choice == '7':
                self.show_image()
            elif choice == '8':
                self.show_sketch()
            elif choice == '9':
                self.show_preview()
            elif choice == '10':
                self.show_statistics()
            elif choice == '11':
                filename = input("Enter filename (or press Enter for default): ").strip()
                self.save_sketch(filename if filename else None)
            elif choice == '12':
                path = input("Enter image path: ").strip()
                self.full_pipeline(path, is_sketch=False)
            elif choice == '13':
                path = input("Enter sketch path: ").strip()
                self.full_pipeline(path, is_sketch=True)
            elif choice == '14':
                break
            else:
                print("Invalid option. Please select 1-14.")


def main():
    """Main entry point."""
    tester = ImageProcessingTester()

    # Check if image/sketch path provided as argument
    if len(sys.argv) > 1:
        file_path = sys.argv[1]

        # Check if --sketch flag is provided
        is_sketch = False
        if len(sys.argv) > 2 and sys.argv[2] == '--sketch':
            is_sketch = True
            print(f"Processing sketch: {file_path}")
        else:
            print(f"Processing image: {file_path}")

        tester.full_pipeline(file_path, is_sketch=is_sketch)
    else:
        # Run interactive menu
        print("\nUsage:")
        print(f"  {sys.argv[0]}                     - Interactive menu")
        print(f"  {sys.argv[0]} <image.jpg>         - Process image")
        print(f"  {sys.argv[0]} <sketch.jpg> --sketch - Process sketch directly")
        print()
        tester.interactive_menu()

    print("\nExiting image processing tester...")


if __name__ == "__main__":
    main()
