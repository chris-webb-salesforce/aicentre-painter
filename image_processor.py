"""
Image processing for sketch creation.
"""

import cv2
import numpy as np
import time
import os
import base64
from config import *


class ImageProcessor:

    def __init__(self):
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
        if USE_OPENAI_SKETCH:
            return self.create_sketch_with_openai()
        else:
            return self.create_sketch_local()

    def create_sketch_with_openai(self):
        """Generate sketch using OpenAI GPT Image."""
        print("Generating sketch using OpenAI...")

        try:
            from openai import OpenAI
        except ImportError:
            print("Error: OpenAI package not installed. Run: pip install openai")
            return self.create_sketch_local()

        api_key = OPENAI_API_KEY or os.getenv('OPENAI_API_KEY')
        if not api_key:
            print("Error: No OpenAI API key found.")
            return self.create_sketch_local()

        try:
            client = OpenAI(api_key=api_key)

            with open(CAPTURED_IMAGE_PATH, 'rb') as image_file:
                result = client.images.edit(
                    model="gpt-image-1",
                    image=image_file,
                    prompt="Transform this portrait into a minimalist single continuous line drawing. Use only simple, clean, flowing black lines on a pure white background. The sketch should be artistic, recognizable, and look like a hand-drawn portrait with minimal detail. No shading, no filling, just clean single line work.",
                    size="1024x1024"
                )

            if hasattr(result.data[0], 'b64_json') and result.data[0].b64_json:
                image_bytes = base64.b64decode(result.data[0].b64_json)
            elif hasattr(result.data[0], 'url') and result.data[0].url:
                import urllib.request
                with urllib.request.urlopen(result.data[0].url) as response:
                    image_bytes = response.read()
            else:
                raise Exception("Unexpected response format from OpenAI")

            from PIL import Image
            from io import BytesIO

            sketch_img = Image.open(BytesIO(image_bytes))
            sketch_np = np.array(sketch_img)

            if len(sketch_np.shape) == 3:
                sketch_cv = cv2.cvtColor(sketch_np, cv2.COLOR_RGB2BGR)
            else:
                sketch_cv = sketch_np

            if len(sketch_cv.shape) == 3:
                gray = cv2.cvtColor(sketch_cv, cv2.COLOR_BGR2GRAY)
            else:
                gray = sketch_cv

            resized = cv2.resize(gray, (IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX))
            _, final_sketch = cv2.threshold(resized, 127, 255, cv2.THRESH_BINARY)

            cv2.imwrite(SKETCH_IMAGE_PATH, final_sketch)
            print(f"OpenAI sketch saved to {SKETCH_IMAGE_PATH}")

            return final_sketch

        except Exception as e:
            print(f"Error generating sketch with OpenAI: {e}")
            print("Falling back to local sketch generation...")
            return self.create_sketch_local()

    def create_sketch_local(self):
        """Convert captured image to sketch using local processing."""
        print("Converting image to sketch...")
        img = cv2.imread(CAPTURED_IMAGE_PATH)
        if img is None:
            print("Error: Could not read captured image.")
            return None

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if self.is_already_line_drawing(gray):
            print("Input appears to be a line drawing")
            final_sketch = self.process_line_drawing(gray)
        else:
            print("Input appears to be a photo")
            final_sketch = self.process_photo_to_sketch(gray)

        cv2.imwrite(SKETCH_IMAGE_PATH, final_sketch)
        print(f"Sketch saved to {SKETCH_IMAGE_PATH}")
        return final_sketch

    def is_already_line_drawing(self, gray_img):
        """Detect if image is already a line drawing."""
        hist = cv2.calcHist([gray_img], [0], None, [256], [0, 256])
        black_pixels = np.sum(hist[0:50])
        white_pixels = np.sum(hist[205:256])
        total_pixels = gray_img.shape[0] * gray_img.shape[1]
        extreme_ratio = (black_pixels + white_pixels) / total_pixels
        return extreme_ratio > 0.7

    def process_line_drawing(self, gray_img):
        """Process an existing line drawing to extract clean lines."""
        _, binary = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        if np.mean(binary) < 127:
            binary = 255 - binary

        inverted = 255 - binary

        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        cleaned = cv2.morphologyEx(inverted, cv2.MORPH_OPEN, kernel_small)

        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
        connected = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel_close)

        return 255 - connected

    def skeletonize(self, img):
        """Thin image to single-pixel width lines."""
        _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        skel = np.zeros(binary.shape, np.uint8)

        done = False
        iterations = 0
        max_iterations = 50

        while not done and iterations < max_iterations:
            eroded = cv2.erode(binary, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(binary, temp)
            skel = cv2.bitwise_or(skel, temp)
            binary = eroded.copy()
            done = (cv2.countNonZero(binary) == 0)
            iterations += 1

        return skel

    def process_photo_to_sketch(self, gray_img):
        """Convert a photograph to sketch using edge detection."""
        simplified = cv2.bilateralFilter(gray_img, 15, 80, 80)
        simplified = cv2.bilateralFilter(simplified, 15, 80, 80)

        blur1 = cv2.GaussianBlur(simplified, (3, 3), 1)
        blur2 = cv2.GaussianBlur(simplified, (7, 7), 2)
        dog = cv2.absdiff(blur1, blur2)
        dog = cv2.multiply(dog, 3.0)

        _, edges = cv2.threshold(dog, 30, 255, cv2.THRESH_BINARY)
        return 255 - edges

    def is_contour_closed(self, points, threshold=10.0):
        """Check if a contour is closed."""
        if len(points) < 3:
            return False
        start = points[0]
        end = points[-1]
        dist = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        return dist < threshold

    def break_closed_contour(self, points):
        """Remove closing segment from closed contours."""
        if len(points) < 4:
            return points
        if self.is_contour_closed(points, threshold=CLOSED_CONTOUR_THRESHOLD):
            return points[:-1]
        return points

    def smooth_contour(self, points, smoothing_factor=3):
        """Smooth contour points using a moving average filter."""
        if len(points) < smoothing_factor * 2:
            return points

        smoothed = []
        half_window = smoothing_factor // 2

        for i in range(len(points)):
            start_idx = max(0, i - half_window)
            end_idx = min(len(points), i + half_window + 1)

            x_avg = sum(p[0] for p in points[start_idx:end_idx]) / (end_idx - start_idx)
            y_avg = sum(p[1] for p in points[start_idx:end_idx]) / (end_idx - start_idx)

            smoothed.append((x_avg, y_avg))

        return smoothed

    def remove_duplicate_contours(self, contours, distance_threshold=5.0):
        """Remove duplicate/overlapping contours."""
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
        """Check if two contours are similar."""
        if abs(len(points1) - len(points2)) > 5:
            return False

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

        return close_count >= sample_size * 0.7

    def preprocess_contours(self, sketch_image):
        """Preprocess and filter contours for drawing."""
        contours, _ = cv2.findContours(sketch_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = []
        closed_count = 0

        for contour in contours:
            try:
                area = cv2.contourArea(contour)
                if area < MIN_CONTOUR_AREA:
                    continue

                epsilon = CONTOUR_SIMPLIFICATION_FACTOR * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if len(approx) >= 2:
                    mm_points = []
                    for point in approx:
                        px_x, px_y = point[0]
                        mm_x = ORIGIN_X + (px_x / IMAGE_WIDTH_PX) * DRAWING_AREA_WIDTH_MM
                        mm_y = ORIGIN_Y + (1.0 - px_y / IMAGE_HEIGHT_PX) * DRAWING_AREA_HEIGHT_MM
                        mm_points.append((mm_x, mm_y))

                    if BREAK_CLOSED_CONTOURS:
                        if self.is_contour_closed(mm_points, threshold=CLOSED_CONTOUR_THRESHOLD):
                            closed_count += 1
                        mm_points = self.break_closed_contour(mm_points)

                    if CONTOUR_SMOOTHING > 0:
                        smoothed_points = self.smooth_contour(mm_points, smoothing_factor=CONTOUR_SMOOTHING)
                    else:
                        smoothed_points = mm_points

                    valid_contours.append((area, smoothed_points))
            except:
                continue

        valid_contours.sort(key=lambda x: x[0], reverse=True)

        if DUPLICATE_CONTOUR_THRESHOLD > 0:
            unique_contours = self.remove_duplicate_contours(valid_contours, distance_threshold=DUPLICATE_CONTOUR_THRESHOLD)
        else:
            unique_contours = valid_contours

        print(f"Contours: {len(contours)} found -> {len(valid_contours)} valid -> {len(unique_contours)} unique")
        if BREAK_CLOSED_CONTOURS and closed_count > 0:
            print(f"  Broke {closed_count} closed contours")

        return [points for _, points in unique_contours]

    def create_contour_preview(self, contours):
        """Create a visual preview of the contours to be drawn."""
        preview_img = np.ones((IMAGE_HEIGHT_PX, IMAGE_WIDTH_PX, 3), dtype=np.uint8) * 255

        colors = [
            (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255),
            (255, 0, 255), (255, 255, 0), (128, 0, 128), (255, 165, 0),
        ]

        for i, contour_points in enumerate(contours):
            if len(contour_points) < 2:
                continue

            color = colors[i % len(colors)]

            pixel_points = []
            for point in contour_points:
                mm_x, mm_y = point[0], point[1]
                px_x = int((mm_x - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX)
                px_y = int((1.0 - (mm_y - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM) * IMAGE_HEIGHT_PX)
                px_x = max(0, min(IMAGE_WIDTH_PX - 1, px_x))
                px_y = max(0, min(IMAGE_HEIGHT_PX - 1, px_y))
                pixel_points.append((px_x, px_y))

            if len(pixel_points) >= 2:
                for j in range(1, len(pixel_points)):
                    cv2.line(preview_img, pixel_points[j-1], pixel_points[j], color, 2)
                cv2.circle(preview_img, pixel_points[0], 4, (0, 0, 0), -1)
                cv2.putText(preview_img, str(i+1), (pixel_points[0][0]+10, pixel_points[0][1]-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        arrow_color = (50, 50, 50)
        for i in range(min(5, len(contours)-1)):
            if len(contours[i]) >= 2 and len(contours[i+1]) >= 2:
                curr_end = contours[i][-1]
                next_start = contours[i+1][0]

                curr_px = (int((curr_end[0] - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX),
                          int((1.0 - (curr_end[1] - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM) * IMAGE_HEIGHT_PX))
                next_px = (int((next_start[0] - ORIGIN_X) / DRAWING_AREA_WIDTH_MM * IMAGE_WIDTH_PX),
                          int((1.0 - (next_start[1] - ORIGIN_Y) / DRAWING_AREA_HEIGHT_MM) * IMAGE_HEIGHT_PX))

                curr_px = (max(0, min(IMAGE_WIDTH_PX-1, curr_px[0])), max(0, min(IMAGE_HEIGHT_PX-1, curr_px[1])))
                next_px = (max(0, min(IMAGE_WIDTH_PX-1, next_px[0])), max(0, min(IMAGE_HEIGHT_PX-1, next_px[1])))

                cv2.arrowedLine(preview_img, curr_px, next_px, arrow_color, 1, tipLength=0.3)

        legend_y = 30
        cv2.putText(preview_img, "Drawing Preview:", (10, legend_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        cv2.putText(preview_img, f"{len(contours)} contours", (10, legend_y + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv2.putText(preview_img, "Press 'd' to draw, 's' to save", (10, legend_y + 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 0), 1)

        return preview_img

    def optimize_contour_path(self, contours):
        """Optimize drawing order to minimize travel."""
        if not contours:
            return []

        remaining = list(contours)
        ordered = []

        current = remaining.pop(0)
        ordered.append(current)
        last_point = current[-1][:2] if len(current) > 0 else [0, 0]

        while remaining:
            closest = None
            min_dist = float('inf')

            for contour in remaining:
                if len(contour) > 0:
                    first_point = contour[0][:2]
                    dist = np.linalg.norm(np.array(last_point) - np.array(first_point))
                    if dist < min_dist:
                        min_dist = dist
                        closest = contour

            if closest is not None:
                ordered.append(closest)
                remaining = [c for c in remaining if c is not closest]
                last_point = closest[-1][:2] if len(closest) > 0 else last_point
            else:
                break

        return ordered

