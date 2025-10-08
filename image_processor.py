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
        """Convert captured image to pencil sketch."""
        print("Converting image to sketch...")
        img = cv2.imread(CAPTURED_IMAGE_PATH)
        if img is None:
            print("Error: Could not read captured image.")
            return None

        # Original pencil sketch method
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        inverted_image = 255 - gray_image
        blurred_image = cv2.GaussianBlur(inverted_image, (21, 21), 0)
        inverted_blurred_image = 255 - blurred_image
        pencil_sketch = cv2.divide(gray_image, inverted_blurred_image, scale=256.0)

        # Adaptive threshold for clean contours
        # Block size: smaller = more detail, larger = simpler
        final_sketch = cv2.adaptiveThreshold(
            pencil_sketch, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            15, 4  # Detail level: 11=detailed, 15=medium, 21=simple
        )

        cv2.imwrite(SKETCH_IMAGE_PATH, final_sketch)
        print(f"Sketch created and saved to {SKETCH_IMAGE_PATH}")
        return final_sketch

    def is_contour_closed(self, points, threshold=10.0):
        """Check if a contour is closed (start and end points are close)."""
        if len(points) < 3:
            return False

        start = points[0]
        end = points[-1]

        dist = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        return dist < threshold  # Increased from 2.0 to 10.0mm

    def break_closed_contour(self, points):
        """
        For closed contours, find the best break point to avoid drawing
        back over the same line.
        Returns points without the closing segment.
        """
        if len(points) < 4:
            return points

        # Remove the last point if it's very close to the first
        # (this is the closing segment - typically blob outlines)
        if self.is_contour_closed(points, threshold=CLOSED_CONTOUR_THRESHOLD):
            return points[:-1]

        return points

    def smooth_contour(self, points, smoothing_factor=3):
        """
        Smooth contour points using a moving average filter.
        Higher smoothing_factor = smoother curves (try 2-5).
        """
        if len(points) < smoothing_factor * 2:
            return points

        smoothed = []
        half_window = smoothing_factor // 2

        for i in range(len(points)):
            # Get window of points around current point
            start_idx = max(0, i - half_window)
            end_idx = min(len(points), i + half_window + 1)

            # Average X and Y coordinates
            x_avg = sum(p[0] for p in points[start_idx:end_idx]) / (end_idx - start_idx)
            y_avg = sum(p[1] for p in points[start_idx:end_idx]) / (end_idx - start_idx)
            z = points[i][2]  # Keep original Z

            smoothed.append((x_avg, y_avg, z))

        return smoothed

    def remove_duplicate_contours(self, contours, distance_threshold=5.0):
        """
        Remove duplicate/overlapping contours that are very close together.
        This prevents drawing the same line twice.
        """
        if len(contours) < 2:
            return contours

        unique_contours = []
        used = set()

        for i, contour1 in enumerate(contours):
            if i in used:
                continue

            is_duplicate = False
            for j, contour2 in enumerate(unique_contours):
                if self.contours_are_similar(contour1[1], contour2[1], distance_threshold):
                    is_duplicate = True
                    break

            if not is_duplicate:
                unique_contours.append(contour1)

        return unique_contours

    def contours_are_similar(self, points1, points2, threshold=5.0):
        """Check if two contours are very similar (likely duplicates)."""
        if abs(len(points1) - len(points2)) > 5:  # Very different point counts
            return False

        # Sample a few points and check if they're close
        sample_size = min(5, len(points1), len(points2))
        sample_indices = [int(i * len(points1) / sample_size) for i in range(sample_size)]

        close_count = 0
        for idx in sample_indices:
            if idx >= len(points1) or idx >= len(points2):
                continue

            p1 = points1[min(idx, len(points1)-1)]
            p2 = points2[min(idx, len(points2)-1)]

            dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            if dist < threshold:
                close_count += 1

        # If most sampled points are close, contours are similar
        return close_count >= sample_size * 0.7

    def preprocess_contours(self, sketch_image):
        """Preprocess and filter contours for detailed drawing."""
        # RETR_LIST gets all contours (both outer and inner)
        # This gives line-like effect instead of just outlines
        # RETR_EXTERNAL would only get outer boundaries (outlines filled areas)
        contours, _ = cv2.findContours(sketch_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Pre-filter and convert all contours at once
        valid_contours = []
        closed_count = 0
        for contour in contours:
            try:
                area = cv2.contourArea(contour)
                if area < MIN_CONTOUR_AREA:  # Skip very small contours
                    continue

                # Simplification to reduce points while preserving shape
                epsilon = CONTOUR_SIMPLIFICATION_FACTOR * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if len(approx) >= 2:  # Need at least 2 points
                    # Pre-convert to mm coordinates with explicit Z coordinate
                    mm_points = []
                    for point in approx:
                        px_x, px_y = point[0]
                        # Map pixel coordinates to mm coordinates
                        # X: 0..IMAGE_WIDTH_PX → ORIGIN_X..ORIGIN_X+WIDTH
                        mm_x = ORIGIN_X + (px_x / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
                        # Y: 0..IMAGE_HEIGHT_PX → ORIGIN_Y+HEIGHT..ORIGIN_Y (inverted)
                        mm_y = ORIGIN_Y + (1.0 - px_y / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
                        mm_z = PEN_DRAWING_Z  # Explicitly set safe drawing height
                        mm_points.append((mm_x, mm_y, mm_z))

                    # Break closed contours to avoid drawing back over the same line
                    if BREAK_CLOSED_CONTOURS:
                        if self.is_contour_closed(mm_points, threshold=CLOSED_CONTOUR_THRESHOLD):
                            closed_count += 1
                        mm_points = self.break_closed_contour(mm_points)

                    # Apply smoothing to reduce jaggedness
                    if CONTOUR_SMOOTHING > 0:
                        smoothed_points = self.smooth_contour(mm_points, smoothing_factor=CONTOUR_SMOOTHING)
                    else:
                        smoothed_points = mm_points

                    valid_contours.append((area, smoothed_points))
            except:
                continue

        # Sort by area (largest first)
        valid_contours.sort(key=lambda x: x[0], reverse=True)

        # Remove duplicate/overlapping contours
        if DUPLICATE_CONTOUR_THRESHOLD > 0:
            unique_contours = self.remove_duplicate_contours(valid_contours, distance_threshold=DUPLICATE_CONTOUR_THRESHOLD)
        else:
            unique_contours = valid_contours

        print(f"Contours: {len(contours)} found → {len(valid_contours)} valid → {len(unique_contours)} unique")
        if BREAK_CLOSED_CONTOURS and closed_count > 0:
            print(f"  Broke {closed_count} closed contours to prevent backtracking")

        return [points for _, points in unique_contours]

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
            for point in contour_points:
                mm_x, mm_y = point[0], point[1]  # Use only X,Y for display (ignore Z)
                # Reverse the coordinate transformation
                px_x = int((mm_x - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX)
                px_y = int((1.0 - (mm_y - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM) * IMAGE_HEIGHT_PX)
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
                
                # Convert to pixels (use X,Y only)
                curr_px = (int((curr_end[0] - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX),
                          int((1.0 - (curr_end[1] - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM) * IMAGE_HEIGHT_PX))
                next_px = (int((next_start[0] - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX),
                          int((1.0 - (next_start[1] - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM) * IMAGE_HEIGHT_PX))
                
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
        cv2.putText(preview_img, "Press 'd' to draw, 's' to save preview", (10, legend_y + 90),
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
        last_point = current[-1][:2] if len(current) > 0 else [0, 0]  # Use X,Y only for distance calc
        
        while remaining:
            closest = None
            min_dist = float('inf')
            
            for contour in remaining:
                if len(contour) > 0:
                    first_point = contour[0][:2]  # Use X,Y only for distance calc
                    dist = np.linalg.norm(np.array(last_point) - np.array(first_point))
                    if dist < min_dist:
                        min_dist = dist
                        closest = contour
            
            if closest is not None:
                ordered.append(closest)
                # Use list comprehension to safely remove the contour
                remaining = [c for c in remaining if c is not closest]
                last_point = closest[-1][:2] if len(closest) > 0 else last_point  # Use X,Y only
            else:
                break
        
        return ordered

    def create_single_line_path(self, contours):
        """Convert multiple contours into a single continuous path."""
        if not contours:
            return []
        
        print("Creating single continuous line path...")
        
        # Start with optimized path order
        optimized_contours = self.optimize_contour_path(contours)
        
        # Connect all contours into one continuous path
        single_path = []
        
        for i, contour_points in enumerate(optimized_contours):
            if len(contour_points) < 1:
                continue
            
            # Add all points from this contour
            single_path.extend(contour_points)
            
            # Add connecting line to next contour (if there is one)
            if i < len(optimized_contours) - 1 and len(optimized_contours[i + 1]) > 0:
                current_end = contour_points[-1]
                next_start = optimized_contours[i + 1][0]
                
                # Add intermediate points for smooth connection if distance is large
                distance = np.sqrt((next_start[0] - current_end[0])**2 + (next_start[1] - current_end[1])**2)
                
                # Add connecting points if jump is > 10mm
                if distance > 10:
                    num_points = max(2, int(distance / 5))  # One point every 5mm
                    for step in range(1, num_points):
                        t = step / num_points
                        interp_x = current_end[0] + t * (next_start[0] - current_end[0])
                        interp_y = current_end[1] + t * (next_start[1] - current_end[1])
                        interp_z = current_end[2]  # Keep same Z
                        single_path.append((interp_x, interp_y, interp_z))
        
        print(f"✅ Single path created: {len(single_path)} points from {len(optimized_contours)} contours")
        return [single_path]  # Return as single contour list