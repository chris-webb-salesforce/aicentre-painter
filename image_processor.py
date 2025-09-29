"""
Image processing module for face detection and sketch creation.
Handles camera capture, face detection, and image-to-sketch conversion.
"""

import cv2
import numpy as np
import time
from config import *


class ImageProcessor:
    """Handle image capture, processing, and sketch generation."""
    
    def __init__(self):
        """Initialize image processor."""
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
            
            if self.face_cascade is not None:
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
                if self.face_cascade is not None and last_face_coords:
                    (x, y, w, h) = last_face_coords
                    padding = 30
                    face_roi = frame[max(0, y-padding):min(frame.shape[0], y+h+padding),
                                     max(0, x-padding):min(frame.shape[1], x+w+padding)]
                    
                    resized_face = cv2.resize(face_roi, (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX))
                    cv2.imwrite(CAPTURED_IMAGE_PATH, resized_face)
                    print(f"Face captured and saved to {CAPTURED_IMAGE_PATH}")
                    break
                else:
                    # No face detection or no face found, capture full frame
                    resized_frame = cv2.resize(frame, (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX))
                    cv2.imwrite(CAPTURED_IMAGE_PATH, resized_frame)
                    print(f"Image captured and saved to {CAPTURED_IMAGE_PATH}")
                    break
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
        cv2.putText(preview_img, "Press 'd' to draw, 's' to save, 'g' for G-code, 'i' for NGC", (10, legend_y + 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 0), 1)
        
        return preview_img

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