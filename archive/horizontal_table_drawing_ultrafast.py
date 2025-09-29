import cv2
import time
from pymycobot import MyCobot320
import numpy as np
import sys

# --- ULTRA-FAST Configuration ---
ORIGIN_X = 200.0
ORIGIN_Y = 0.0
ORIGIN_Z = 100.0

DRAWING_AREA_WIDTH_MM = 120
DRAWING_AREA_HEIGHT_MM = 180

# Ultra-aggressive pressure settings
PEN_DRAWING_Z = ORIGIN_Z - 3
PEN_RETRACT_Z = ORIGIN_Z + 10  # Reduced lift height

# Maximum speed settings
APPROACH_SPEED = 80
DRAWING_SPEED = 100  # Maximum safe speed
LIFT_SPEED = 100
TRAVEL_SPEED = 100

# Minimal processing
CONTOUR_SIMPLIFICATION_FACTOR = 0.05  # Very aggressive simplification
MIN_CONTOUR_AREA = 50  # Skip small details
MAX_CONTOURS = 200  # Limit total contours

SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

IMAGE_WIDTH_PX = 400
IMAGE_HEIGHT_PX = 600
SKETCH_IMAGE_PATH = "sketch_to_draw.jpg"

class UltraFastDrawingRobot:
    def __init__(self):
        print("Initializing ULTRA-FAST Drawing Robot...")
        self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
        time.sleep(1)
        self.mc.set_fresh_mode(1)
        
        self.DRAWING_ORIENTATION = [180, 0, 45]
        self.pen_is_down = False
    
    def ultra_fast_pen_down(self, x, y):
        """Instant pen down with no intermediate steps."""
        if not self.pen_is_down:
            self.mc.send_coords([x, y, PEN_DRAWING_Z] + self.DRAWING_ORIENTATION, APPROACH_SPEED, 0)
            time.sleep(0.05)  # Absolute minimum delay
            self.pen_is_down = True
    
    def ultra_fast_pen_up(self):
        """Instant pen up."""
        if self.pen_is_down:
            current = self.mc.get_coords()
            if current:
                self.mc.send_coords([current[0], current[1], PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, LIFT_SPEED, 0)
                time.sleep(0.03)  # Minimal delay
            self.pen_is_down = False
    
    def ultra_preprocess_contours(self, sketch_image):
        """Ultra-fast contour preprocessing."""
        contours, _ = cv2.findContours(sketch_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process only the largest contours
        contour_data = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= MIN_CONTOUR_AREA:
                contour_data.append((area, contour))
        
        # Sort and limit
        contour_data.sort(key=lambda x: x[0], reverse=True)
        contour_data = contour_data[:MAX_CONTOURS]
        
        # Convert to mm coordinates with maximum simplification
        result = []
        for area, contour in contour_data:
            epsilon = CONTOUR_SIMPLIFICATION_FACTOR * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) >= 2:
                mm_points = []
                for point in approx:
                    px_x, px_y = point[0]
                    mm_x = ORIGIN_X + (px_x / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
                    mm_y = ORIGIN_Y + DRAWING_AREA_HEIGHT_MM - (px_y / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
                    mm_points.append((mm_x, mm_y))
                result.append(mm_points)
        
        return result
    
    def ultra_fast_draw(self, sketch_image):
        """Ultra-fast drawing with absolute minimal overhead."""
        if sketch_image is None:
            print("Cannot draw, sketch image is missing.")
            return
        
        print("ULTRA-FAST MODE: Processing contours...")
        contours = self.ultra_preprocess_contours(sketch_image)
        print(f"Drawing {len(contours)} contours at maximum speed...")
        
        # Immediate start position
        self.mc.send_coords([ORIGIN_X, ORIGIN_Y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, TRAVEL_SPEED, 0)
        time.sleep(0.5)
        
        start_time = time.time()
        
        # Draw with absolute minimum processing
        for i, contour_points in enumerate(contours):
            if len(contour_points) < 2:
                continue
            
            # Progress every 100 contours
            if i % 100 == 0:
                print(f"Progress: {i+1}/{len(contours)}")
            
            # Ultra-fast pen operations
            start_x, start_y = contour_points[0]
            self.ultra_fast_pen_down(start_x, start_y)
            
            # Draw entire contour without individual segment delays
            for next_x, next_y in contour_points[1:]:
                self.mc.send_coords([next_x, next_y, PEN_DRAWING_Z] + self.DRAWING_ORIENTATION, DRAWING_SPEED, 0)
                # NO time.sleep() for maximum speed
            
            self.ultra_fast_pen_up()
        
        elapsed_time = time.time() - start_time
        print(f"\\n🚀 ULTRA-FAST Drawing complete in {elapsed_time//60:.0f}m {elapsed_time%60:.0f}s!")
        print(f"Average: {elapsed_time/len(contours):.3f}s per contour")
    
    def create_sketch(self):
        """Convert captured image to sketch (same as original)."""
        print("Converting image to sketch...")
        img = cv2.imread('captured_face.jpg')
        if img is None:
            print("Error: Could not read captured image.")
            return None
        
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        inverted_image = 255 - gray_image
        blurred_image = cv2.GaussianBlur(inverted_image, (21, 21), 0)
        inverted_blurred_image = 255 - blurred_image
        pencil_sketch = cv2.divide(gray_image, inverted_blurred_image, scale=256.0)
        
        final_sketch = cv2.adaptiveThreshold(pencil_sketch, 255,
                                             cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.THRESH_BINARY_INV, 11, 2)
        
        cv2.imwrite(SKETCH_IMAGE_PATH, final_sketch)
        return final_sketch
    
    def run(self):
        """Ultra-fast execution."""
        print("\\n🚀 ULTRA-FAST Drawing Robot 🚀")
        print("WARNING: This mode prioritizes speed over drawing quality!")
        
        sketch = self.create_sketch()
        if sketch is not None:
            cv2.imshow("Sketch - Press 'd' for ULTRA-FAST draw", sketch)
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            if key == ord('d'):
                self.ultra_fast_draw(sketch)
        
        print("Ultra-fast mode complete.")

if __name__ == "__main__":
    robot = UltraFastDrawingRobot()
    robot.run()