import cv2
import time
from pymycobot import MyCobot320
import numpy as np
import sys
import os

# --- Workspace Calibration for HORIZONTAL Drawing ---
# The (X, Y, Z) coordinate of the top-left corner of your drawing area on flat table
# These values are for table surface drawing
ORIGIN_X = 200.0  # X position on table
ORIGIN_Y = 0.0    # Y position on table  
ORIGIN_Z = 100.0  # Height above table (adjust based on your table height)

# Drawing area dimensions
DRAWING_AREA_WIDTH_MM = 120
DRAWING_AREA_HEIGHT_MM = 180

# --- Pressure Control Settings ---
# For flat table drawing, Z controls pen height
PEN_CONTACT_Z = ORIGIN_Z - 2   # Just touching the paper
PEN_DRAWING_Z = ORIGIN_Z - 3   # Drawing pressure (lower = more pressure)
PEN_RETRACT_Z = ORIGIN_Z + 30  # Safe height above paper

# --- Movement Control Settings ---
# Speed settings for different operations
APPROACH_SPEED = 15  # Slower speed when approaching paper
DRAWING_SPEED = 20  # Slower speed while drawing for better quality
LIFT_SPEED = 30  # Speed when lifting pen
TRAVEL_SPEED = 40  # Speed for non-drawing movements

# Movement interpolation settings
INTERPOLATION_POINTS = 5  # More intermediate points for smoother curves
MIN_SEGMENT_LENGTH = 1.5  # Smaller segments for better detail

# --- Force Protection Settings ---
MAX_DRAWING_FORCE = 5  # Maximum force to apply (robot units)
FORCE_CHECK_INTERVAL = 0.1  # How often to check force feedback

# --- Robot Configuration ---
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

# --- Image Configuration ---
IMAGE_WIDTH_PX = 400
IMAGE_HEIGHT_PX = 600

# --- Drawing Optimization ---
CONTOUR_SIMPLIFICATION_FACTOR = 0.008  # Slightly higher for smoother lines
REST_INTERVAL = 30  # Rest every N contours
REST_DURATION_S = 1
OPTIMIZE_DRAWING_PATH = True

# --- File Paths ---
CAPTURED_IMAGE_PATH = "captured_face.jpg"
SKETCH_IMAGE_PATH = "sketch_to_draw.jpg"
HAAR_CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
CAMERA_INDEX = 0

class HorizontalTableDrawingRobot:
    def __init__(self):
        print("Initializing Horizontal Table Drawing Robot...")
        try:
            self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
            time.sleep(2)  # Give robot time to initialize
            
            # Enable adaptive mode for smoother movements
            self.mc.set_fresh_mode(1)  # Enable coordinate refresh mode
            
        except Exception as e:
            print(f"\\n--- ERROR ---")
            print(f"Failed to connect to robot: {e}")
            sys.exit(1)
    
    def force_j6_angle(self):
        """Force J6 to the desired angle while maintaining current position."""
        try:
            current_joints = self.mc.get_angles()
            if current_joints and len(current_joints) >= 6:
                # Keep all joints the same except J6
                current_joints[5] = self.DESIRED_J6_ANGLE  # J6 is index 5 (0-based)
                self.mc.send_angles(current_joints, 20)
                time.sleep(0.2)
        except:
            pass  # Ignore errors, continue drawing
        
        print("Loading face detection model...")
        self.face_cascade = cv2.CascadeClassifier(HAAR_CASCADE_PATH)
        
        # Orientation for flat table drawing with pen pointing straight down
        # [RX, RY, RZ] - pen holder rotation controlled by J6 joint position
        self.DRAWING_ORIENTATION = [180, 0, 0]
        
        # Track current pen state
        self.pen_is_down = False
        self.current_position = None
        
        # Desired J6 angle for pen holder
        self.DESIRED_J6_ANGLE = 45
        
    def calibrate_pen_pressure(self):
        """Interactive calibration to find optimal pen pressure."""
        print("\\n--- PEN PRESSURE CALIBRATION ---")
        print("This will help find the optimal pen pressure for your setup.")
        print("Place a test paper on the vertical surface.")
        
        self.go_to_home_position()
        
        # Move to center of drawing area
        test_x = ORIGIN_X + DRAWING_AREA_WIDTH_MM / 2
        test_z = ORIGIN_Z - DRAWING_AREA_HEIGHT_MM / 2
        
        print("Moving to test position...")
        self.mc.send_coords([test_x, test_y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, 30, 0)
        time.sleep(3)
        
        test_y = ORIGIN_Y
        step_size = 0.5
        
        print("\\nUse keyboard to adjust pen pressure:")
        print("  'f' = Move pen forward (more pressure)")
        print("  'b' = Move pen backward (less pressure)")
        print("  's' = Save this position")
        print("  'q' = Cancel calibration")
        
        while True:
            print(f"Current Y position: {test_y:.1f}mm")
            self.mc.send_coords([test_x, test_y, test_z] + self.DRAWING_ORIENTATION, 15, 0)
            time.sleep(0.5)
            
            key = input("Command: ").lower().strip()
            
            if key == 'f':
                test_y += step_size
                print("Moving forward...")
            elif key == 'b':
                test_y -= step_size
                print("Moving backward...")
            elif key == 's':
                print(f"\\nOptimal pressure Y position saved: {test_y:.1f}mm")
                print("Update PEN_DRAWING_Y in your code to this value.")
                self.mc.send_coords([test_x, test_y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, 30, 0)
                time.sleep(2)
                return test_y
            elif key == 'q':
                print("Calibration cancelled.")
                self.mc.send_coords([test_x, test_y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, 30, 0)
                time.sleep(2)
                return None
                
    def smooth_approach(self, target_coords, speed=APPROACH_SPEED):
        """Gradually approach the target position to avoid sudden impacts."""
        current = self.mc.get_coords()
        if not current:
            self.mc.send_coords(target_coords, speed, 0)
            return
        
        # Create intermediate waypoints for smooth approach
        steps = 3
        for i in range(1, steps + 1):
            ratio = i / steps
            intermediate = [
                current[0] + (target_coords[0] - current[0]) * ratio,
                current[1] + (target_coords[1] - current[1]) * ratio,
                current[2] + (target_coords[2] - current[2]) * ratio,
            ] + target_coords[3:]
            
            self.mc.send_coords(intermediate, speed, 0)
            time.sleep(0.1)
    
    def gentle_pen_down(self, x, y):
        """Gently lower the pen to the drawing surface."""
        if self.pen_is_down:
            return
            
        # First move to position above the paper
        self.mc.send_coords([x, y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, TRAVEL_SPEED, 0)
        time.sleep(0.8)  # Longer delay for position settling
        self.force_j6_angle()  # Ensure pen holder angle
        
        # Approach slowly to contact position
        self.smooth_approach([x, y, PEN_CONTACT_Z] + self.DRAWING_ORIENTATION, APPROACH_SPEED)
        time.sleep(0.3)
        self.force_j6_angle()  # Maintain pen holder angle
        
        # Apply gentle drawing pressure
        self.mc.send_coords([x, y, PEN_DRAWING_Z] + self.DRAWING_ORIENTATION, APPROACH_SPEED // 2, 0)
        time.sleep(0.4)  # Longer delay for pressure settling
        self.force_j6_angle()  # Ensure pen holder angle
        
        self.pen_is_down = True
        self.current_position = [x, y]
    
    def gentle_pen_up(self):
        """Gently lift the pen from the drawing surface."""
        if not self.pen_is_down:
            return
            
        current = self.mc.get_coords()
        if current:
            # First reduce pressure
            self.mc.send_coords([current[0], current[1], PEN_CONTACT_Z] + self.DRAWING_ORIENTATION, LIFT_SPEED, 0)
            time.sleep(0.2)
            self.force_j6_angle()
            
            # Then retract
            self.mc.send_coords([current[0], current[1], PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, LIFT_SPEED, 0)
            time.sleep(0.5)
            self.force_j6_angle()
        
        self.pen_is_down = False
    
    def draw_line_segment(self, from_point, to_point):
        """Draw a line segment with smooth interpolation."""
        x1, y1 = from_point
        x2, y2 = to_point
        
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if distance < MIN_SEGMENT_LENGTH:
            # Direct movement for short segments
            self.mc.send_coords([x2, y2, PEN_DRAWING_Z] + self.DRAWING_ORIENTATION, DRAWING_SPEED, 0)
            time.sleep(0.1)  # Increased delay
            self.force_j6_angle()  # Maintain pen angle
        else:
            # Interpolate for smooth curves
            num_points = max(2, int(distance / MIN_SEGMENT_LENGTH))
            for i in range(1, num_points + 1):
                ratio = i / num_points
                x = x1 + (x2 - x1) * ratio
                y = y1 + (y2 - y1) * ratio
                self.mc.send_coords([x, y, PEN_DRAWING_Z] + self.DRAWING_ORIENTATION, DRAWING_SPEED, 0)
                time.sleep(0.08)  # Increased delay for smoother drawing
                # Force J6 angle every few points to avoid drift
                if i % 3 == 0:  # Every 3rd point
                    self.force_j6_angle()
        
        self.current_position = [x2, y2]
    
    def go_to_home_position(self):
        """Move to drawing start position (previously home)."""
        print("Moving to drawing start position...")
        self.gentle_pen_up()
        # Position robot ready for drawing on flat table
        initial_coords = [ORIGIN_X, ORIGIN_Y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION
        self.mc.send_coords(initial_coords, 40, 0)
        time.sleep(3)
        self.force_j6_angle()  # Ensure correct pen holder angle
    
    def go_to_photo_position(self):
        """Move to photo capture position."""
        print("Moving to photo position...")
        self.mc.send_angles([0, -45, -45, 0, 90, 0], 40)
        time.sleep(3)
    
    def go_to_safe_position(self):
        """Move to neutral safe position when not drawing."""
        print("Moving to safe neutral position...")
        self.gentle_pen_up()
        self.mc.send_angles([0, 0, 0, 0, 90, self.DESIRED_J6_ANGLE], 40)  # J6 at desired angle
        time.sleep(3)
    
    def test_drawing_area(self):
        """Test the drawing area with gentle movements."""
        print("\\n--- TESTING DRAWING AREA ---")
        print("This will draw a test rectangle to verify settings.")
        
        self.go_to_home_position()
        
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
        self.mc.send_coords([start_x, start_y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, TRAVEL_SPEED, 0)
        time.sleep(2)
        
        # Draw test rectangle
        print("Drawing test rectangle...")
        self.gentle_pen_down(start_x, start_y)
        
        for i in range(1, len(test_corners)):
            x, y = test_corners[i]
            print(f"Drawing edge {i}/{len(test_corners)-1}...")
            self.draw_line_segment(test_corners[i-1], test_corners[i])
            time.sleep(0.2)
        
        self.gentle_pen_up()
        print("Test complete!")
        self.go_to_home_position()
    
    def capture_image(self):
        """Capture face image from camera."""
        print("Preparing camera...")
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            print(f"Error: Cannot open camera at index {CAMERA_INDEX}")
            return False
        
        last_face_coords = None
        
        while True:
            ret, frame = cap.read()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            if not ret:
                print("Error: Failed to grab frame.")
                break
            
            preview_frame = frame.copy()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray_frame, 1.3, 5)
            
            if len(faces) > 0:
                (x, y, w, h) = faces[0]
                last_face_coords = (x, y, w, h)
                cv2.rectangle(preview_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            else:
                last_face_coords = None
            
            cv2.imshow('Camera - Press "c" to capture, "q" to quit', preview_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                if last_face_coords:
                    (x, y, w, h) = last_face_coords
                    padding = 30
                    face_roi = frame[max(0, y-padding):min(frame.shape[0], y+h+padding),
                                     max(0, x-padding):min(frame.shape[1], x+w+padding)]
                    
                    resized_face = cv2.resize(face_roi, (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX))
                    cv2.imwrite(CAPTURED_IMAGE_PATH, resized_face)
                    print(f"Face captured and saved to {CAPTURED_IMAGE_PATH}")
                    break
                else:
                    print("No face detected! Please try again.")
            elif key == ord('q'):
                print("Quitting capture.")
                cap.release()
                cv2.destroyAllWindows()
                return False
        
        cap.release()
        cv2.destroyAllWindows()
        return True
    
    def create_sketch(self):
        """Convert captured image to sketch."""
        print("Converting image to sketch...")
        img = cv2.imread(CAPTURED_IMAGE_PATH)
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
        print(f"Sketch created and saved to {SKETCH_IMAGE_PATH}")
        return final_sketch
    
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
    
    def optimize_contour_path(self, contours):
        """Optimize drawing order to minimize travel."""
        if not contours:
            return []
        
        print("Optimizing drawing path...")
        remaining = list(contours)
        ordered = []
        
        # Start with largest contour
        current = remaining.pop(0)
        ordered.append(current)
        last_point = current[-1][0] if len(current) > 0 else [0, 0]
        
        while remaining:
            closest = None
            min_dist = float('inf')
            
            for contour in remaining:
                if len(contour) > 0:
                    first_point = contour[0][0]
                    dist = np.linalg.norm(last_point - first_point)
                    if dist < min_dist:
                        min_dist = dist
                        closest = contour
            
            if closest is not None:
                ordered.append(closest)
                # Use list comprehension to safely remove the contour
                remaining = [c for c in remaining if c is not closest]
                last_point = closest[-1][0] if len(closest) > 0 else last_point
            else:
                break
        
        return ordered
    
    def draw_sketch(self, sketch_image):
        """Draw the sketch with optimized vertical drawing."""
        if sketch_image is None:
            print("Cannot draw, sketch image is missing.")
            return
        
        # Find and simplify contours
        contours, _ = cv2.findContours(sketch_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        simplified_contours = []
        for contour in contours:
            # Skip if contour is too small or invalid
            try:
                area = cv2.contourArea(contour)
                if area < 2:
                    continue
            except:
                continue
            
            epsilon = CONTOUR_SIMPLIFICATION_FACTOR * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            simplified_contours.append(approx)
        
        # Sort by area and optimize path
        contours = sorted(simplified_contours, key=cv2.contourArea, reverse=True)
        
        if OPTIMIZE_DRAWING_PATH:
            contours = self.optimize_contour_path(contours)
        
        print(f"Found {len(contours)} contours to draw.")
        print("="*60)  # Separator line
        
        # Move to safe starting position
        self.go_to_home_position()
        initial_coords = [ORIGIN_X, ORIGIN_Y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION
        self.mc.send_coords(initial_coords, TRAVEL_SPEED, 0)
        time.sleep(3)
        
        total_contours = len(contours)
        contours_drawn = 0
        start_time = time.time()
        
        for i, contour in enumerate(contours):
            if len(contour) < 2:
                continue
            
            # Rest interval
            if REST_INTERVAL > 0 and contours_drawn > 0 and contours_drawn % REST_INTERVAL == 0:
                print()  # New line before rest message
                print(f"\\n💤 Resting for {REST_DURATION_S} seconds...")
                self.gentle_pen_up()
                self.go_to_home_position()
                time.sleep(REST_DURATION_S)
                self.mc.send_coords(initial_coords, TRAVEL_SPEED, 0)
                time.sleep(2)
                print()  # Resume drawing message
            
            # Update progress bar
            self.print_progress_bar(i+1, total_contours, prefix="Drawing")
            
            # Convert first point to mm (maintain top-to-bottom, left-to-right tracing)
            start_point_px = contour[0][0]
            start_x_mm = ORIGIN_X + (start_point_px[0] / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
            start_y_mm = ORIGIN_Y + DRAWING_AREA_HEIGHT_MM - (start_point_px[1] / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
            
            # Move to start position and lower pen
            self.gentle_pen_down(start_x_mm, start_y_mm)
            
            # Draw the contour with smooth segments
            prev_point = (start_x_mm, start_y_mm)
            for j in range(1, len(contour)):
                point_px = contour[j][0]
                x_mm = ORIGIN_X + (point_px[0] / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
                y_mm = ORIGIN_Y + DRAWING_AREA_HEIGHT_MM - (point_px[1] / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
                
                self.draw_line_segment(prev_point, (x_mm, y_mm))
                prev_point = (x_mm, y_mm)
            
            # Lift pen for next contour
            self.gentle_pen_up()
            contours_drawn += 1
        
        # Final progress bar update and completion message
        print()  # New line after progress bar
        elapsed_time = time.time() - start_time
        print("="*60)
        print(f"\\n✓ Drawing complete!")
        print(f"  Total contours drawn: {contours_drawn}")
        print(f"  Time elapsed: {elapsed_time//60:.0f}m {elapsed_time%60:.0f}s")
        print("="*60)
        self.go_to_home_position()
    
    def run(self):
        """Main execution loop."""
        print("\\n--- MyCobot Vertical Drawing System ---")
        print("Optimized for gentle vertical surface drawing\\n")
        
        # Offer calibration option
        calibrate = input("Would you like to calibrate pen pressure? (y/n): ").lower().strip()
        if calibrate == 'y':
            optimal_y = self.calibrate_pen_pressure()
            if optimal_y:
                global PEN_DRAWING_Y
                PEN_DRAWING_Y = optimal_y
        
        # Test drawing area
        test = input("Would you like to test the drawing area? (y/n): ").lower().strip()
        if test == 'y':
            self.test_drawing_area()
        
        while True:
            action = input("\\nPress ENTER to start drawing (or 'exit' to quit): ").lower().strip()
            if action == 'exit':
                break
            
            # Photo capture
            # Skip for now
            # self.go_to_photo_position()
            # if not self.capture_image():
            #     print("Image capture cancelled.")
            #     self.go_to_home_position()
            #     continue
            
            # Create sketch
            sketch = self.create_sketch()
            if sketch is not None:
                cv2.imshow("Generated Sketch - Press 'd' to draw, any key to cancel", sketch)
                key = cv2.waitKey(0)
                cv2.destroyAllWindows()
                
                if key == ord('d'):
                    self.draw_sketch(sketch)
                else:
                    print("Drawing cancelled.")
                    self.go_to_home_position()
            else:
                print("Failed to create sketch.")
        
        print("Shutting down.")

if __name__ == "__main__":
    if not os.path.exists(HAAR_CASCADE_PATH):
        print(f"\\n--- ERROR ---")
        print("Haar Cascade file not found.")
        print(f"Path: {HAAR_CASCADE_PATH}")
    else:
        robot = HorizontalTableDrawingRobot()
        robot.run()