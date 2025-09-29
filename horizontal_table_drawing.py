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
# For flat table drawing, Z controls pen height (OPTIMIZED FOR STABILITY)
PEN_CONTACT_Z = ORIGIN_Z - 1.5   # Light contact to test surface
PEN_DRAWING_Z = ORIGIN_Z - 2.5   # Moderate drawing pressure (reduced to prevent digging)
PEN_RETRACT_Z = ORIGIN_Z + 25    # Safe height above paper (reduced for faster movement)

# --- Movement Control Settings ---
# Speed settings for different operations (STABILIZED FOR REDUCED WOBBLE)
APPROACH_SPEED = 20  # Slower approach for better control
DRAWING_SPEED = 25  # Reduced drawing speed to minimize wobble
LIFT_SPEED = 30  # Controlled pen lifting
TRAVEL_SPEED = 40  # Moderate travel to reduce vibration

# Movement interpolation settings (STABILIZED)
INTERPOLATION_POINTS = 4  # More interpolation points for smoother curves
MIN_SEGMENT_LENGTH = 3.0  # Smaller segments for better precision
MOVEMENT_SETTLING_TIME = 0.05  # Time to let arm settle between movements

# --- Force Protection and Depth Control Settings ---
MAX_DRAWING_FORCE = 5  # Maximum force to apply (robot units)
FORCE_CHECK_INTERVAL = 0.1  # How often to check force feedback
Z_COMPENSATION_FACTOR = 0.995  # Very gradual Z compensation to prevent digging
Z_STABILIZATION_THRESHOLD = 0.3  # Tighter threshold for Z-axis correction
MAX_Z_DRIFT = 0.8  # Maximum allowed Z drift before correction

# --- Robot Configuration ---
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

# --- Image Configuration ---
IMAGE_WIDTH_PX = 400
IMAGE_HEIGHT_PX = 600

# --- Drawing Optimization (PERFORMANCE OPTIMIZED) ---
# OLD VALUES (comment/uncomment to switch):
# CONTOUR_SIMPLIFICATION_FACTOR = 0.02  # Higher simplification for fewer points
# MIN_CONTOUR_AREA = 10  # Skip very small contours

# MEDIUM SIMPLIFICATION (faster drawing, decent quality):
CONTOUR_SIMPLIFICATION_FACTOR = 0.05  # More aggressive - fewer points
MIN_CONTOUR_AREA = 25  # Skip more small details

# FOR EVEN FASTER (uncomment these instead):
# CONTOUR_SIMPLIFICATION_FACTOR = 0.1  # Very aggressive
# MIN_CONTOUR_AREA = 50  # Skip most small details

REST_INTERVAL = 0  # Disable rest intervals for continuous drawing
REST_DURATION_S = 0
OPTIMIZE_DRAWING_PATH = True
BATCH_SIZE = 10  # Process contours in batches

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
        
        print("Loading face detection model...")
        try:
            self.face_cascade = cv2.CascadeClassifier(HAAR_CASCADE_PATH)
            if self.face_cascade.empty():
                print("Warning: Face cascade not loaded properly, continuing without face detection")
                self.face_cascade = None
            else:
                print("Face detection model loaded successfully")
        except Exception as e:
            print(f"Warning: Could not load face detection model: {e}")
            self.face_cascade = None
        
        # Orientation for flat table drawing with pen pointing straight down
        # [RX, RY, RZ] - pen holder rotation controlled by J6 joint position
        self.DRAWING_ORIENTATION = [180, 0, 45]
        
        # Track current pen state and depth
        self.pen_is_down = False
        self.current_position = None
        self.current_z_depth = PEN_RETRACT_Z
        self.movement_counter = 0
        
        
        # Orientation for flat table drawing with pen pointing straight down
        # [RX, RY, RZ] - pen holder rotation controlled by J6 joint position
        self.DRAWING_ORIENTATION = [180, 0, 45]
        
        # Track current pen state
        self.pen_is_down = False
        self.current_position = None
        
        # Desired J6 angle for pen holder
        self.DESIRED_J6_ANGLE = 45
        
        # Force feedback monitoring
        self.last_force_check = time.time()
        self.force_warnings = 0
    
    def check_force_feedback(self):
        """Monitor force feedback and adjust Z if needed."""
        current_time = time.time()
        if current_time - self.last_force_check < FORCE_CHECK_INTERVAL:
            return
        
        self.last_force_check = current_time
        
        # Note: Force feedback implementation depends on robot model capabilities
        # This is a placeholder for force monitoring logic
        try:
            # In a real implementation, you would check robot force sensors here
            # For now, we'll use position-based depth compensation
            if self.pen_is_down and self.movement_counter > 75:
                # After many movements, check for excessive drift
                z_drift = PEN_DRAWING_Z - self.current_z_depth
                if abs(z_drift) > Z_STABILIZATION_THRESHOLD:
                    # Apply gentle correction
                    correction = z_drift * 0.1  # 10% correction
                    self.current_z_depth += correction
                    # Clamp to safe range
                    self.current_z_depth = max(PEN_DRAWING_Z - 0.8, 
                                              min(PEN_DRAWING_Z + 0.4, self.current_z_depth))
                    print(f"Force-based Z correction: {self.current_z_depth:.2f}mm (drift: {z_drift:.2f}mm)")
        except Exception as e:
            # Force feedback not available - continue with position control
            pass
    
    def stabilize_arm_position(self):
        """Add a brief pause to let arm stabilize and reduce oscillation."""
        # Get current position to check if arm has settled
        current = self.mc.get_coords()
        if current:
            # Brief stabilization pause
            time.sleep(0.02)
        
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
        """Gently lower the pen to the drawing surface with smooth transition."""
        if self.pen_is_down:
            return
            
        # Move to position above the point
        self.mc.send_coords([x, y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION, TRAVEL_SPEED, 0)
        time.sleep(0.3)  # Allow arm to stabilize
        
        # Gradual approach to drawing surface in steps
        approach_steps = 3
        for i in range(1, approach_steps + 1):
            z_position = PEN_RETRACT_Z - ((PEN_RETRACT_Z - PEN_DRAWING_Z) * (i / approach_steps))
            self.mc.send_coords([x, y, z_position] + self.DRAWING_ORIENTATION, int(APPROACH_SPEED // 2), 0)
            time.sleep(0.1)  # Small settling time between steps
        
        # Final positioning at drawing depth
        self.mc.send_coords([x, y, PEN_DRAWING_Z] + self.DRAWING_ORIENTATION, int(APPROACH_SPEED // 3), 0)
        time.sleep(0.2)  # Final settling time
        
        self.pen_is_down = True
        self.current_position = [x, y]
        self.current_z_depth = PEN_DRAWING_Z
        self.movement_counter = 0
    
    def gentle_pen_up(self):
        """Gently lift the pen from the drawing surface with smooth transition."""
        if not self.pen_is_down:
            return
            
        current = self.mc.get_coords()
        if current:
            # Gradual lift in steps to prevent jerky movement
            lift_steps = 2
            for i in range(1, lift_steps + 1):
                z_position = self.current_z_depth + ((PEN_RETRACT_Z - self.current_z_depth) * (i / lift_steps))
                self.mc.send_coords([current[0], current[1], z_position] + self.DRAWING_ORIENTATION, int(LIFT_SPEED // 2), 0)
                time.sleep(0.08)
        
        self.pen_is_down = False
        self.current_z_depth = PEN_RETRACT_Z
    
    def draw_line_segment(self, from_point, to_point):
        """Draw a line segment with smooth interpolation and depth control."""
        x1, y1 = from_point
        x2, y2 = to_point
        
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Enhanced Z compensation to prevent digging deeper
        self.movement_counter += 1
        if self.movement_counter % 15 == 0:  # Every 15 movements (less frequent)
            # Check if we've drifted too far down
            z_drift = PEN_DRAWING_Z - self.current_z_depth
            if z_drift > MAX_Z_DRIFT:
                # Gradual correction back towards target depth
                self.current_z_depth = (self.current_z_depth * Z_COMPENSATION_FACTOR + 
                                       PEN_DRAWING_Z * (1 - Z_COMPENSATION_FACTOR))
                # Clamp to safe range
                self.current_z_depth = max(PEN_DRAWING_Z - 0.5, 
                                          min(PEN_DRAWING_Z + 0.3, self.current_z_depth))
        
        if distance < MIN_SEGMENT_LENGTH:
            # For short segments, single smooth movement
            self.mc.send_coords([x2, y2, self.current_z_depth] + self.DRAWING_ORIENTATION, DRAWING_SPEED, 0)
            time.sleep(MOVEMENT_SETTLING_TIME)  # Consistent settling time
        else:
            # More interpolation points for smoother curves
            num_points = max(2, min(INTERPOLATION_POINTS, int(distance / MIN_SEGMENT_LENGTH)))
            for i in range(1, num_points + 1):
                ratio = i / num_points
                x = x1 + (x2 - x1) * ratio
                y = y1 + (y2 - y1) * ratio
                
                # Smooth speed ramping for better control
                if i == 1 or i == num_points:
                    speed = int(DRAWING_SPEED * 0.8)  # Slower at segment endpoints
                else:
                    speed = DRAWING_SPEED
                
                self.mc.send_coords([x, y, self.current_z_depth] + self.DRAWING_ORIENTATION, speed, 0)
                time.sleep(MOVEMENT_SETTLING_TIME)  # Consistent settling time
                
                # Check force feedback and stabilize periodically
                if i % 2 == 0:  # Every other point
                    self.check_force_feedback()
                    self.stabilize_arm_position()
        
        self.current_position = [x2, y2]
    
    def go_to_home_position(self):
        """Move to drawing start position (previously home)."""
        print("Moving to drawing start position...")
        self.gentle_pen_up()
        # Position robot ready for drawing on flat table
        initial_coords = [ORIGIN_X, ORIGIN_Y, PEN_RETRACT_Z] + self.DRAWING_ORIENTATION
        self.mc.send_coords(initial_coords, 40, 0)
        time.sleep(3)
        # Pen holder angle set by Cartesian RZ=45
    
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
        
        # Sketch detail level (comment/uncomment to switch):
        # final_sketch = cv2.adaptiveThreshold(pencil_sketch, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)  # DETAILED
        final_sketch = cv2.adaptiveThreshold(pencil_sketch, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 4)  # MEDIUM
        # final_sketch = cv2.adaptiveThreshold(pencil_sketch, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 6)  # SIMPLE
        
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
    
    def preprocess_contours(self, sketch_image):
        """OPTIMIZED: Preprocess and filter contours for faster drawing."""
        contours, _ = cv2.findContours(sketch_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        # Pre-filter and convert all contours at once
        valid_contours = []
        for contour in contours:
            try:
                area = cv2.contourArea(contour)
                if area < MIN_CONTOUR_AREA:  # Skip very small contours
                    continue
                    
                # Aggressive simplification for speed
                epsilon = CONTOUR_SIMPLIFICATION_FACTOR * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                if len(approx) >= 2:  # Need at least 2 points
                    # Pre-convert to mm coordinates
                    mm_points = []
                    for point in approx:
                        px_x, px_y = point[0]
                        mm_x = ORIGIN_X + (px_x / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
                        mm_y = ORIGIN_Y + DRAWING_AREA_HEIGHT_MM - (px_y / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
                        mm_points.append((mm_x, mm_y))
                    valid_contours.append((area, mm_points))
            except:
                continue
        
        # Sort by area (largest first) and extract points
        valid_contours.sort(key=lambda x: x[0], reverse=True)
        return [points for _, points in valid_contours]

    def create_contour_preview(self, contours):
        """Create a visual preview of the contours to be drawn."""
        # Create a blank white image for preview
        preview_img = np.ones((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX, 3), dtype=np.uint8) * 255
        
        # Draw each contour in a different color
        colors = [
            (0, 0, 255),    # Red
            (0, 255, 0),    # Green  
            (255, 0, 0),    # Blue
            (0, 255, 255),  # Yellow
            (255, 0, 255),  # Magenta
            (255, 255, 0),  # Cyan
            (128, 0, 128),  # Purple
            (255, 165, 0),  # Orange
        ]
        
        print(f"Creating preview for {len(contours)} contours...")
        
        for i, contour_points in enumerate(contours):
            if len(contour_points) < 2:
                continue
                
            color = colors[i % len(colors)]
            
            # Convert mm coordinates back to pixel coordinates for display
            pixel_points = []
            for mm_x, mm_y in contour_points:
                # Reverse the coordinate transformation
                px_x = int((mm_x - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX)
                px_y = int(IMAGE_HEIGHT_PX - (mm_y - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM * IMAGE_HEIGHT_PX)
                # Clamp to image bounds
                px_x = max(0, min(IMAGE_WIDTH_PX - 1, px_x))
                px_y = max(0, min(IMAGE_HEIGHT_PX - 1, px_y))
                pixel_points.append((px_x, px_y))
            
            # Draw the contour as connected lines
            if len(pixel_points) >= 2:
                for j in range(1, len(pixel_points)):
                    cv2.line(preview_img, pixel_points[j-1], pixel_points[j], color, 2)
                
                # Draw start point as a circle
                cv2.circle(preview_img, pixel_points[0], 4, (0, 0, 0), -1)
                
                # Add contour number
                text_pos = pixel_points[0]
                cv2.putText(preview_img, str(i+1), (text_pos[0]+10, text_pos[1]-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Add drawing order arrows for first few contours
        arrow_color = (50, 50, 50)
        for i in range(min(5, len(contours)-1)):
            if len(contours[i]) >= 2 and len(contours[i+1]) >= 2:
                # Get end of current contour and start of next
                curr_end = contours[i][-1]
                next_start = contours[i+1][0]
                
                # Convert to pixels
                curr_px = (int((curr_end[0] - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX),
                          int(IMAGE_HEIGHT_PX - (curr_end[1] - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM * IMAGE_HEIGHT_PX))
                next_px = (int((next_start[0] - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX),
                          int(IMAGE_HEIGHT_PX - (next_start[1] - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM * IMAGE_HEIGHT_PX))
                
                # Clamp coordinates
                curr_px = (max(0, min(IMAGE_WIDTH_PX-1, curr_px[0])), max(0, min(IMAGE_HEIGHT_PX-1, curr_px[1])))
                next_px = (max(0, min(IMAGE_WIDTH_PX-1, next_px[0])), max(0, min(IMAGE_HEIGHT_PX-1, next_px[1])))
                
                # Draw dashed line to show travel path
                cv2.arrowedLine(preview_img, curr_px, next_px, arrow_color, 1, tipLength=0.3)
        
        # Add legend
        legend_y = 30
        cv2.putText(preview_img, "Drawing Preview:", (10, legend_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.putText(preview_img, f"• {len(contours)} contours", (10, legend_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(preview_img, "• Black dots = start points", (10, legend_y + 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(preview_img, "• Arrows = travel paths", (10, legend_y + 65), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(preview_img, "Press 'd' to draw, 's' to save, any key to cancel", (10, legend_y + 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 0), 1)
        
        return preview_img

    def draw_sketch(self, sketch_image):
        """OPTIMIZED: Draw the sketch with performance improvements."""
        if sketch_image is None:
            print("Cannot draw, sketch image is missing.")
            return
        
        # Preprocess all contours at once
        print("Preprocessing contours...")
        contours = self.preprocess_contours(sketch_image)
        
        if OPTIMIZE_DRAWING_PATH:
            print("Optimizing path...")
            # Simple nearest-neighbor optimization for pre-converted points
            if contours:
                optimized = [contours[0]]
                remaining = contours[1:]
                current_end = contours[0][-1] if contours[0] else (0, 0)
                
                while remaining:
                    closest_idx = 0
                    min_dist = float('inf')
                    for i, contour in enumerate(remaining):
                        if contour:
                            dist = ((current_end[0] - contour[0][0])**2 + (current_end[1] - contour[0][1])**2)**0.5
                            if dist < min_dist:
                                min_dist = dist
                                closest_idx = i
                    
                    closest = remaining.pop(closest_idx)
                    optimized.append(closest)
                    current_end = closest[-1] if closest else current_end
                contours = optimized
        
        # Create and show preview
        print("Creating drawing preview...")
        preview_img = self.create_contour_preview(contours)
        cv2.imshow("Drawing Preview - Check path and order", preview_img)
        
        print("\nPreview Controls:")
        print("  'd' = Start drawing")
        print("  's' = Save preview image") 
        print("  Any other key = Cancel drawing")
        
        key = cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        if key == ord('s'):
            preview_filename = "drawing_preview.jpg"
            cv2.imwrite(preview_filename, preview_img)
            print(f"Preview saved as {preview_filename}")
            print("Press 'd' to draw or any other key to cancel:")
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        if key != ord('d'):
            print("Drawing cancelled.")
            return
        
        print(f"Drawing {len(contours)} optimized contours...")
        print("="*60)
        
        # Setup
        self.go_to_home_position()
        time.sleep(1)  # Reduced setup time
        
        start_time = time.time()
        
        # OPTIMIZED: Draw all contours with minimal overhead
        for i, contour_points in enumerate(contours):
            if len(contour_points) < 2:
                continue
            
            # Progress update every 50 contours instead of every contour
            if i % 50 == 0:
                self.print_progress_bar(i+1, len(contours), prefix="Drawing")
            
            # Start drawing this contour
            start_x, start_y = contour_points[0]
            self.gentle_pen_down(start_x, start_y)
            
            # Draw all segments in this contour with controlled movements
            for j in range(1, len(contour_points)):
                prev_x, prev_y = contour_points[j-1]
                next_x, next_y = contour_points[j]
                
                # Use proper line segment drawing for better control
                self.draw_line_segment((prev_x, prev_y), (next_x, next_y))
            
            # Lift pen for next contour
            self.gentle_pen_up()
        
        # Final progress and timing
        elapsed_time = time.time() - start_time
        print()
        print("="*60)
        print(f"✓ OPTIMIZED Drawing complete!")
        print(f"  Contours drawn: {len(contours)}")
        print(f"  Time elapsed: {elapsed_time//60:.0f}m {elapsed_time%60:.0f}s")
        print(f"  Average: {elapsed_time/len(contours):.3f}s per contour")
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