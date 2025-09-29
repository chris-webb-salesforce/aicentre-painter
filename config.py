"""
Configuration settings for the AI Centre Painter robot system.
Centralized configuration management for all modules.
"""

import cv2

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

# --- Home Position Storage ---
HOME_POSITION_FILE = "home_position.json"

# --- Drawing Board Dimensions (based on ElephantRobotics workflow) ---
# They suggest 270x400mm drawing area in Inkscape
INKSCAPE_DRAWING_WIDTH = 270  # mm
INKSCAPE_DRAWING_HEIGHT = 400  # mm

# --- Image Configuration ---
IMAGE_WIDTH_PX = 400
IMAGE_HEIGHT_PX = 600

# --- Drawing Optimization (PERFORMANCE OPTIMIZED) ---
# MEDIUM SIMPLIFICATION (faster drawing, decent quality):
CONTOUR_SIMPLIFICATION_FACTOR = 0.05  # More aggressive - fewer points
MIN_CONTOUR_AREA = 25  # Skip more small details

REST_INTERVAL = 0  # Disable rest intervals for continuous drawing
REST_DURATION_S = 0
OPTIMIZE_DRAWING_PATH = True
BATCH_SIZE = 10  # Process contours in batches

# --- File Paths ---
CAPTURED_IMAGE_PATH = "captured_face.jpg"
SKETCH_IMAGE_PATH = "sketch_to_draw.jpg"
HAAR_CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
CAMERA_INDEX = 0

# --- Robot Orientation ---
# Orientation for flat table drawing with pen pointing straight down
# [RX, RY, RZ] - pen holder rotation controlled by J6 joint position
DRAWING_ORIENTATION = [180, 0, 45]

# --- Movement Synchronization Settings ---
USE_MOVEMENT_SYNC = True  # Enable proper movement waiting
MAX_WAIT_TIME = 3.0  # Maximum time to wait for movement completion
POSITION_TOLERANCE = 2.0  # mm tolerance for position checking (looser for speed)
POSITION_CHECK_INTERVAL = 0.02  # Check position every 20ms

# Desired J6 angle for pen holder
DESIRED_J6_ANGLE = 45