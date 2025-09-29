import cv2
import time
from pymycobot.mycobot import MyCobot320
import numpy as np
import sys
import os

# --- Workspace Calibration ---
# The (X, Y, Z) coordinate of the top-left corner of your VERTICAL drawing area.
# You must get these values by running the calibration scripts.
ORIGIN_X = 205.1
ORIGIN_Y = -85.3
ORIGIN_Z = 110.2

# The width and height of your drawing paper in millimeters.
DRAWING_AREA_WIDTH_MM = 120
DRAWING_AREA_HEIGHT_MM = 180

# The distance (in mm) the pen should retract from the wall when moving.
SAFE_RETRACTION_MM = 40

# --- Robot & Connection Configuration ---
SERIAL_PORT = "/dev/ttyAMA0" # For Raspberry Pi
BAUD_RATE = 115200

# -- Image & Drawing Configuration --
IMAGE_WIDTH_PX = 400
IMAGE_HEIGHT_PX = 600

# -- Drawing Performance Configuration --
CONTOUR_SIMPLIFICATION_FACTOR = 0.005
REST_INTERVAL = 50
REST_DURATION_S = 5
# --- NEW: Path Optimization ---
# If True, the robot will intelligently sort the contours to minimize travel time.
OPTIMIZE_DRAWING_PATH = True
# The speed for "travel" moves (when the pen is up). Should be faster than drawing speed.
TRAVEL_SPEED = 60


# -- File Paths --
CAPTURED_IMAGE_PATH = "captured_face.jpg"
SKETCH_IMAGE_PATH = "sketch_to_draw.jpg"
HAAR_CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
CAMERA_INDEX = 0

class MyCobotArtist:
    def __init__(self):
        print("Initializing MyCobot Artist...")
        try:
            self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
        except Exception as e:
            print(f"\n--- ERROR ---")
            print(f"Failed to connect to the robot on port '{SERIAL_PORT}'.")
            print("Please check the connection and ensure the port is correct.")
            print(f"Details: {e}")
            print("---------------")
            sys.exit(1)

        print("Loading face detection model...")
        self.face_cascade = cv2.CascadeClassifier(HAAR_CASCADE_PATH)
        print("Face detection model loaded.")

        # Define a single, stable orientation for all drawing operations.
        # [Rx, Ry, Rz] -> [180, -90, -90] generally points the pen horizontally forward.
        self.STABLE_ORIENTATION = [180, -90, -90]


    def go_to_home_position(self):
        """Moves the robot to a safe, neutral position."""
        print("Moving to home position...")
        # A neutral, upward position.
        self.mc.send_angles([0, 0, 0, 0, 90, 0], 50)
        time.sleep(3)

    def go_to_photo_position(self):
        """Moves the robot to a position ideal for taking a photo."""
        print("Moving to photo position...")
        # A position looking slightly down and forward.
        self.mc.send_angles([0, -45, -45, 0, 90, 0], 50)
        time.sleep(3)

    def lift_pen(self):
        """Lifts the pen to the safe retraction distance from the wall."""
        current_coords = self.mc.get_coords()
        if current_coords:
            current_coords[1] = ORIGIN_Y - SAFE_RETRACTION_MM # Retract along Y-axis
            self.mc.send_coords(current_coords, TRAVEL_SPEED, 0)
            time.sleep(1)

    def outline_drawing_area(self):
        """Traces the perimeter of the vertical drawing area for confirmation."""
        print("Outlining the VERTICAL drawing area for confirmation...")

        # --- NEW: Pre-flight check for workspace boundaries ---
        print("Performing pre-flight check on coordinates...")
        max_reach = 310 # A safe maximum reach in mm for a MyCobot 320
        corners = {
            "top-left": (ORIGIN_X, ORIGIN_Y, ORIGIN_Z),
            "top-right": (ORIGIN_X + DRAWING_AREA_WIDTH_MM, ORIGIN_Y, ORIGIN_Z),
            "bottom-right": (ORIGIN_X + DRAWING_AREA_WIDTH_MM, ORIGIN_Y, ORIGIN_Z - DRAWING_AREA_HEIGHT_MM),
            "bottom-left": (ORIGIN_X, ORIGIN_Y, ORIGIN_Z - DRAWING_AREA_HEIGHT_MM)
        }

        for name, (x, y, z) in corners.items():
            # Check the planar distance from the robot's base
            distance = np.sqrt(x**2 + y**2)
            if distance > max_reach:
                print("\n--- COORDINATE ERROR! ---")
                print(f"The '{name}' corner at (X:{x}, Y:{y}) is likely out of the robot's reach ({distance:.1f}mm > {max_reach}mm).")
                print("Please move your drawing surface closer to the robot and re-calibrate your ORIGIN coordinates.")
                print("--------------------------\n")
                return False
        print("Pre-flight check passed. All coordinates are within safe reach.")
        # --- End of new code ---

        self.go_to_home_position()

        # Define the four corners in the vertical X/Z plane
        tl = [ORIGIN_X, ORIGIN_Y, ORIGIN_Z]
        tr = [ORIGIN_X + DRAWING_AREA_WIDTH_MM, ORIGIN_Y, ORIGIN_Z]
        br = [ORIGIN_X + DRAWING_AREA_WIDTH_MM, ORIGIN_Y, ORIGIN_Z - DRAWING_AREA_HEIGHT_MM]
        bl = [ORIGIN_X, ORIGIN_Y, ORIGIN_Z - DRAWING_AREA_HEIGHT_MM]

        # Move to a safe start position (retracted from the wall)
        safe_start = [tl[0], tl[1] - SAFE_RETRACTION_MM, tl[2]]
        print(f"Moving to safe start position... Target Coords: {safe_start}")
        self.mc.send_coords(safe_start + self.STABLE_ORIENTATION, 40, 0)
        time.sleep(3)

        # Move pen to wall at top-left
        print(f"Moving pen to wall at top-left... Target Coords: {tl}")
        self.mc.send_coords(tl + self.STABLE_ORIENTATION, 25, 0)
        time.sleep(1)

        # Draw the rectangle using synchronous calls to ensure completion
        print(f"Drawing top edge...  Target Coords: {tr}")
        self.mc.sync_send_coords(tr + self.STABLE_ORIENTATION, 30, 7)

        print(f"Drawing right edge... Target Coords: {br}")
        self.mc.sync_send_coords(br + self.STABLE_ORIENTATION, 30, 7)

        print(f"Drawing bottom edge... Target Coords: {bl}")
        self.mc.sync_send_coords(bl + self.STABLE_ORIENTATION, 30, 7)

        print(f"Drawing left edge...  Target Coords: {tl}")
        self.mc.sync_send_coords(tl + self.STABLE_ORIENTATION, 30, 7)

        # Lift pen
        print("Lifting pen...")
        self.mc.send_coords([tl[0], tl[1] - SAFE_RETRACTION_MM, tl[2]] + self.STABLE_ORIENTATION, 40, 0)
        time.sleep(2)
        self.go_to_home_position()
        return True

    def capture_image(self):
        """Opens a camera feed, detects a face, and captures it."""
        print("Preparing camera...")
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            print(f"Error: Cannot open camera at index {CAMERA_INDEX}")
            return False

        last_face_coords = None

        while True:
            ret, frame = cap.read()
            # ADD THIS LINE TO ROTATE THE CAMERA FEED 180 DEGREES
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            if not ret:
                print("Error: Failed to grab frame.")
                break

            # Create a copy of the frame for display so the original is not modified
            preview_frame = frame.copy()

            # Create a grayscale version for face detection
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray_frame, 1.3, 5)

            # Draw rectangle around the face in the preview
            if len(faces) > 0:
                # Get the first face detected
                (x, y, w, h) = faces[0]
                last_face_coords = (x, y, w, h)
                # Draw a green rectangle on the preview frame, not the original
                cv2.rectangle(preview_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            else:
                last_face_coords = None

            cv2.imshow('Camera - Press "c" to capture, "q" to quit', preview_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                if last_face_coords:
                    (x, y, w, h) = last_face_coords
                    # Add some padding around the face
                    padding = 30
                    # IMPORTANT: Crop from the original, unmodified 'frame'
                    face_roi = frame[max(0, y-padding):min(frame.shape[0], y+h+padding),
                                     max(0, x-padding):min(frame.shape[1], x+w+padding)]

                    # Save the captured face
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
        """
        Converts the captured image into a black-and-white sketch using the
        "color dodge" blending method.
        """
        print("Converting image to sketch using pencil sketch technique...")
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

    def _optimize_path(self, contours):
        """Sorts contours using a nearest-neighbor approach to minimize travel time."""
        print("Optimizing drawing path...")
        
        # Guard against empty list
        if not contours:
            return []

        # Convert contours to a list for easier manipulation
        remaining_contours = list(contours)
        sorted_contours = []

        # Start with the first contour found
        current_contour = remaining_contours.pop(0)
        sorted_contours.append(current_contour)

        # The last point of the current path
        last_point = current_contour[-1][0]

        while remaining_contours:
            closest_contour = None
            closest_distance = float('inf')
            
            # Find the nearest contour to the last point of the previous one
            for contour in remaining_contours:
                first_point = contour[0][0]
                distance = np.linalg.norm(last_point - first_point)
                
                if distance < closest_distance:
                    closest_distance = distance
                    closest_contour = contour
            
            # Add the closest contour to our sorted list
            sorted_contours.append(closest_contour)
            remaining_contours.remove(closest_contour)
            last_point = closest_contour[-1][0]
        
        print(f"Path optimized. Reordered {len(sorted_contours)} contours.")
        return sorted_contours

    def draw_sketch(self, sketch_image):
        """Finds contours and sends them to the robot to draw on the VERTICAL plane."""
        if sketch_image is None:
            print("Cannot draw, sketch image is missing.")
            return

        contours, _ = cv2.findContours(sketch_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        simplified_contours = []
        if CONTOUR_SIMPLIFICATION_FACTOR > 0:
            for contour in contours:
                epsilon = CONTOUR_SIMPLIFICATION_FACTOR * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                simplified_contours.append(approx)
            contours = simplified_contours
        
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # --- NEW: Optimize the drawing order ---
        if OPTIMIZE_DRAWING_PATH:
            contours = self._optimize_path(contours)

        print(f"Found {len(contours)} lines to draw. Starting drawing process...")

        drawing_y = ORIGIN_Y
        safe_y = ORIGIN_Y - SAFE_RETRACTION_MM
        
        # Move to a safe starting position above the top-left corner
        initial_coords = [ORIGIN_X, safe_y, ORIGIN_Z] + self.STABLE_ORIENTATION
        self.mc.send_coords(initial_coords, TRAVEL_SPEED, 0) # Use faster travel speed
        time.sleep(3)

        total_contours = len(contours)
        for i, contour in enumerate(contours):
            if cv2.contourArea(contour) < 2:
                continue

            if REST_INTERVAL > 0 and i > 0 and i % REST_INTERVAL == 0:
                print(f"\n--- Pausing for {REST_DURATION_S} seconds to cool down servos... ---")
                self.lift_pen()
                self.go_to_home_position()
                self.mc.send_coords(initial_coords, TRAVEL_SPEED, 0) # Use faster travel speed
                time.sleep(2)

            print(f"Drawing line {i+1}/{total_contours}...")

            start_point_px = contour[0][0]
            start_x_mm = ORIGIN_X + (start_point_px[0] / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
            start_z_mm = ORIGIN_Z - (start_point_px[1] / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
            
            # Move to start of contour at a safe distance
            self.mc.send_coords([start_x_mm, safe_y, start_z_mm] + self.STABLE_ORIENTATION, TRAVEL_SPEED, 0) # Use faster travel speed
            time.sleep(0.5)

            # Move pen to wall
            self.mc.send_coords([start_x_mm, drawing_y, start_z_mm] + self.STABLE_ORIENTATION, 30, 0)
            time.sleep(0.5)

            # Draw the contour
            for point in contour[1:]:
                point_px = point[0]
                x_mm = ORIGIN_X + (point_px[0] / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
                z_mm = ORIGIN_Z - (point_px[1] / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
                self.mc.send_coords([x_mm, drawing_y, z_mm] + self.STABLE_ORIENTATION, 30, 0)
                time.sleep(0.08)

            # Retract pen from wall
            self.mc.send_coords([x_mm, safe_y, z_mm] + self.STABLE_ORIENTATION, TRAVEL_SPEED, 0)
            time.sleep(0.5)

        print("\nDrawing complete!")
        self.go_to_home_position()

    def run(self):
        """Main execution loop for the project."""
        print("\n--- MyCobot Sketch Artist ---")
        
        # 1. Outline the workspace
        if not self.outline_drawing_area():
            print("Workspace outlining failed or was aborted. Exiting.")
            return
            
        while True:
            action = input("Press ENTER to start (or type 'exit' to quit): ").lower().strip()
            if action == 'exit':
                break
            
            # 2. Go to photo position
            self.go_to_photo_position()

            # 3. Capture an image
            if not self.capture_image():
                print("Image capture was cancelled. Ready for next command.")
                self.go_to_home_position()
                continue
            
            # 4. Create the sketch
            sketch = self.create_sketch()
            
            # 5. Show sketch and ask for confirmation
            if sketch is not None:
                cv2.imshow("Generated Sketch - Press 'd' to draw, any other key to cancel", sketch)
                key = cv2.waitKey(0)
                cv2.destroyAllWindows()
                
                if key == ord('d'):
                    # 6. Draw the sketch
                    self.draw_sketch(sketch)
                else:
                    print("Drawing cancelled by user.")
                    self.go_to_home_position()
            else:
                print("Failed to create sketch.")

        print("Shutting down.")

if __name__ == "__main__":
    if not os.path.exists(HAAR_CASCADE_PATH):
        print(f"\n--- ERROR ---")
        print("Haar Cascade file not found for face detection.")
        print(f"Please ensure the path is correct: {HAAR_CASCADE_PATH}")
        print("---------------")
    else:
        artist = MyCobotArtist()
        artist.run()

