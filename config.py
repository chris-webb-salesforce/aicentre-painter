"""
Configuration settings for the AI Centre Painter robot system.
SIMPLIFIED - Core settings only for reliable drawing.
"""

import cv2

# =============================================================================
# ROBOT CONNECTION
# =============================================================================
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

# =============================================================================
# DRAWING WORKSPACE
# =============================================================================
# The (X, Y, Z) coordinate of the origin of your drawing area
ORIGIN_X = 190.0  # Moved closer to robot base (was 200.0)
ORIGIN_Y = 0.0
ORIGIN_Z = 10.0

# Drawing area dimensions (mm)
# Reduced to ensure all contours fit within robot's safe workspace
DRAWING_AREA_WIDTH_MM = 100  # Was 120 - reduced to prevent overhang
DRAWING_AREA_HEIGHT_MM = 100  # Was 120 - keeping square aspect ratio

# =============================================================================
# PEN HEIGHTS - FIXED (No compensation)
# =============================================================================
SAFE_TRAVEL_HEIGHT = ORIGIN_Z + 50      # Pen fully up for travel
PEN_RETRACT_Z = ORIGIN_Z + 25           # Pen up between contours
PEN_DRAWING_Z = ORIGIN_Z - 2.5          # Pen touching paper (drawing)

# =============================================================================
# SAFETY LIMITS
# =============================================================================
MIN_SAFE_Z = ORIGIN_Z - 10              # Absolute minimum Z
MAX_SAFE_Z = ORIGIN_Z + 200             # Maximum safe Z height
SAFETY_MARGIN = 5                       # Additional safety margin (mm)

# =============================================================================
# MOVEMENT SPEEDS - SLOW AND RELIABLE
# =============================================================================
TRAVEL_SPEED = 20       # Moving with pen up
DRAWING_SPEED = 10      # Drawing lines
APPROACH_SPEED = 15     # Lowering pen
LIFT_SPEED = 20         # Raising pen

# =============================================================================
# TIMING - CRITICAL FOR RELIABILITY
# =============================================================================
MIN_COMMAND_INTERVAL = 0.5          # Minimum time between commands (seconds)
MOVEMENT_TIMEOUT = 5.0              # Max wait for movement completion
POSITION_TOLERANCE = 3.0            # Position accuracy tolerance (mm)
POSITION_CHECK_INTERVAL = 0.05      # How often to check position (seconds)
MOVEMENT_SETTLING_TIME = 0.1        # Time to let arm settle

# =============================================================================
# MOVEMENT SYNCHRONIZATION - MUST BE TRUE
# =============================================================================
USE_MOVEMENT_SYNC = True            # Wait for each movement to complete
MAX_WAIT_TIME = 5.0                 # Maximum wait time per move

# =============================================================================
# IMAGE PROCESSING
# =============================================================================
IMAGE_WIDTH_PX = 400
IMAGE_HEIGHT_PX = 600

# Contour filtering
MIN_CONTOUR_AREA = 20                       # Reduced to keep more detail (was 50)
CONTOUR_SIMPLIFICATION_FACTOR = 0.003       # Less simplification for smoother lines (was 0.01)
CONTOUR_SMOOTHING = 5                       # Smoothing strength: 2=light, 3=medium, 5=heavy, 0=off

# Path optimization
OPTIMIZE_DRAWING_PATH = True                # Use nearest-neighbor ordering

# =============================================================================
# CAMERA AND FILE PATHS
# =============================================================================
CAPTURED_IMAGE_PATH = "captured_face.jpg"
SKETCH_IMAGE_PATH = "sketch_to_draw.jpg"
HAAR_CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
CAMERA_INDEX = 0

# =============================================================================
# ROBOT ORIENTATION
# =============================================================================
# [RX, RY, RZ] - Pen pointing straight down for flat table drawing
DRAWING_ORIENTATION = [180, 0, 45]

# Desired J6 angle for pen holder
DESIRED_J6_ANGLE = 0
