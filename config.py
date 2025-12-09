"""
Configuration settings for the AI Centre Painter robot system.
"""

import cv2
import os
from pathlib import Path

# Load environment variables from .env file if it exists
try:
    from dotenv import load_dotenv
    env_path = Path(__file__).parent / '.env'
    load_dotenv(dotenv_path=env_path)
except ImportError:
    # python-dotenv not installed, will fall back to os.getenv()
    pass

# =============================================================================
# ROBOT CONNECTION
# =============================================================================
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

# =============================================================================
# CALIBRATION DATA
# =============================================================================
CALIBRATION_FILE = "surface_calibration.json"

# =============================================================================
# DRAWING WORKSPACE
# =============================================================================
ORIGIN_X = 220.0
ORIGIN_Y = 0.0
ORIGIN_Z = 10.0

DRAWING_AREA_WIDTH_MM = 100
DRAWING_AREA_HEIGHT_MM = 100

# =============================================================================
# PEN HEIGHT OFFSETS (relative to calibrated surface)
# =============================================================================
LIFT_HEIGHT_OFFSET = 25.0
DRAWING_HEIGHT_OFFSET = -2.0

# =============================================================================
# SAFE TRAVEL HEIGHT
# =============================================================================
SAFE_TRAVEL_HEIGHT = ORIGIN_Z + 50

# =============================================================================
# SAFETY LIMITS
# =============================================================================
MIN_SAFE_Z = ORIGIN_Z - 10
MAX_SAFE_Z = ORIGIN_Z + 200
SAFETY_MARGIN = 5

# =============================================================================
# MOVEMENT SPEEDS
# =============================================================================
TRAVEL_SPEED = 30
DRAWING_SPEED = 15
APPROACH_SPEED = 20
LIFT_SPEED = 25

# =============================================================================
# TIMING
# =============================================================================
MIN_COMMAND_INTERVAL = 0.2
MOVEMENT_TIMEOUT = 3.0
POSITION_TOLERANCE = 5.0
MOVEMENT_SETTLING_TIME = 0.05
POSITION_CHECK_INTERVAL = 0.05

# =============================================================================
# MOVEMENT SYNCHRONIZATION
# =============================================================================
USE_MOVEMENT_SYNC = True
MAX_WAIT_TIME = 3.0

# =============================================================================
# IMAGE PROCESSING
# =============================================================================
IMAGE_WIDTH_PX = 480
IMAGE_HEIGHT_PX = 480

MIN_CONTOUR_AREA = 30
CONTOUR_SIMPLIFICATION_FACTOR = 0.002
CONTOUR_SMOOTHING = 3
DUPLICATE_CONTOUR_THRESHOLD = 5.0
BREAK_CLOSED_CONTOURS = True
CLOSED_CONTOUR_THRESHOLD = 10.0
OPTIMIZE_DRAWING_PATH = True

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
DRAWING_ORIENTATION = [180, 0, 45]
DESIRED_J6_ANGLE = 0

# =============================================================================
# OPENAI API SETTINGS
# =============================================================================
OPENAI_API_KEY = os.getenv('OPENAI_API_KEY', None)
USE_OPENAI_SKETCH = os.getenv('USE_OPENAI_SKETCH', 'False').lower() in ('true', '1', 'yes')
