# MyCobot320 Drawing Robot - Simplification Plan

## Current Problem
The robot is **skipping instructions** and **breaking pens** due to:
- Command queue overrun (sending too fast)
- Over-complicated optimization pipeline
- Movement sync disabled but code assumes it's working
- Z-compensation causing sudden height changes mid-draw
- Too many layers fighting each other

## Goal
**Simple**: Take photo → Create sketch → Draw it reliably

**That's it.** No G-code, no exports, no PyBullet, no optimization layers.

---

## Phase 1: Delete Unnecessary Code

### Files to DELETE entirely
- `gcode_handler.py` - Not needed, we're drawing directly
- Any G-code/NGC export functionality
- PyBullet simulation code (if exists)
- Trajectory export features

### Features to REMOVE from existing files

#### From `config.py`:
- All G-code settings (lines 95-123)
- Inkscape compatibility settings (lines 77-79)
- Path optimization flags (keep only `OPTIMIZE_DRAWING_PATH = False`)
- Distance compensation (set `ENABLE_DISTANCE_COMPENSATION = False`)
- Adaptive speed settings
- Path smoothing settings

#### From `robot_controller.py`:
- `interpolate_linear_path()` - Too complex
- `move_linear_interpolated()` - Not needed
- `calculate_distance_compensation()` - Breaks pens
- `apply_z_compensation()` - Causes issues
- All force feedback code - Not available on hardware
- `test_distance_compensation()` - Remove
- `calibrate_compensation_interactively()` - Remove

#### From `main_app.py`:
- G-code execution options (menu items 2, 3)
- All export functionality
- Trajectory export handling
- Home position recording (unless actually needed)
- Initial setup complexity - make it simple defaults

#### From `image_processor.py` (need to check this file):
- Douglas-Peucker simplification
- Path smoothing
- 2-opt optimization
- Complex path ordering (keep only simple nearest-neighbor)

---

## Phase 2: Core Functionality - Keep Only This

### What We Actually Need

```
1. Robot Connection
   ↓
2. Capture Image (or load test image)
   ↓
3. Convert to Sketch (edge detection)
   ↓
4. Extract Contours
   ↓
5. Draw Contours (simple, reliable)
   ↓
6. Done
```

### Drawing Loop (The Heart of Everything)

```python
def draw_sketch(contours):
    """Simple, reliable drawing - no optimizations."""

    robot.go_to_home()
    wait(2)  # Let it settle

    for contour in contours:
        if len(contour) < 2:
            continue

        # Move to start (pen up)
        robot.pen_up()
        robot.move_to(contour[0], safe_height=True)
        wait(1)  # CRITICAL: Wait for position

        # Lower pen
        robot.pen_down()
        wait(1)  # CRITICAL: Wait for pen contact

        # Draw each point
        for point in contour[1:]:
            robot.draw_to(point)
            wait(0.5)  # CRITICAL: Wait for completion

        # Lift pen
        robot.pen_up()
        wait(1)  # CRITICAL: Wait for lift

    robot.go_to_home()
```

**Key principle**: WAIT after EVERY command until robot confirms completion.

---

## Phase 3: New Configuration (Simple)

### Core Settings Only

```python
# Robot Connection
SERIAL_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200

# Drawing Area
ORIGIN_X = 200.0
ORIGIN_Y = 0.0
ORIGIN_Z = 10.0
DRAWING_WIDTH = 120
DRAWING_HEIGHT = 180

# Pen Heights (FIXED - no compensation)
SAFE_HEIGHT = ORIGIN_Z + 50     # Pen fully up
DRAWING_HEIGHT_Z = ORIGIN_Z - 2.5  # Pen touching paper

# Speeds (SLOW and RELIABLE)
TRAVEL_SPEED = 20   # Moving with pen up
DRAWING_SPEED = 10  # Drawing lines
PEN_SPEED = 15      # Pen up/down movements

# Safety
MIN_SAFE_Z = ORIGIN_Z - 10
MAX_SAFE_Z = ORIGIN_Z + 200

# Timing (CRITICAL)
MIN_COMMAND_INTERVAL = 0.5  # Half second between commands
MOVEMENT_TIMEOUT = 5.0      # Max wait for completion
STABILIZATION_DELAY = 0.5   # Let arm settle

# Movement Sync
USE_MOVEMENT_SYNC = True  # MUST BE TRUE

# Image Processing (Simple)
IMAGE_WIDTH = 400
IMAGE_HEIGHT = 600
MIN_CONTOUR_AREA = 50  # Skip tiny details
SIMPLIFICATION = 0.01   # Minimal simplification only
```

---

## Phase 4: Simplified Robot Controller

### Core Methods Only

```python
class RobotController:
    def __init__(self):
        # Connect to robot
        # Set fresh mode
        # Initialize state

    def move_to(self, x, y, z=None):
        """Move to position and WAIT for completion."""
        # Validate coordinates
        # Send command
        # WAIT for robot to reach position
        # Return success/failure

    def pen_down(self):
        """Lower pen to drawing height."""
        # Get current X,Y
        # Move Z down slowly
        # WAIT for completion
        # Set pen_is_down = True

    def pen_up(self):
        """Raise pen to safe height."""
        # Get current X,Y
        # Move Z up
        # WAIT for completion
        # Set pen_is_down = False

    def draw_to(self, x, y):
        """Draw line to point (pen must be down)."""
        # Move to X,Y at drawing height
        # WAIT for completion

    def go_to_home(self):
        """Move to home position."""
        # Pen up first
        # Move to origin
        # WAIT for completion

    def wait_for_completion(self, timeout=5.0):
        """ACTUALLY wait until robot stops moving."""
        # Poll position until stable
        # Or timeout
        # Return success/failure
```

---

## Phase 5: Implementation Steps

### Step 1: Backup Current Code
```bash
git commit -am "backup before simplification"
git branch backup-complex-version
```

### Step 2: Delete Files
- Delete `gcode_handler.py`
- Remove imports from `main_app.py`

### Step 3: Simplify `config.py`
- Keep only settings listed in Phase 3
- Remove all G-code, optimization, compensation settings

### Step 4: Simplify `robot_controller.py`
- Remove all compensation code
- Remove interpolation code
- Keep only: move_to, pen_up, pen_down, draw_to, go_to_home
- FIX `wait_for_completion()` to ACTUALLY wait

### Step 5: Simplify `image_processor.py`
- Keep only: load image, create sketch, extract contours
- Remove all optimization
- Simple contour extraction only

### Step 6: Simplify `main_app.py`
- Remove G-code menu options
- Remove export options
- Single flow: Image → Sketch → Draw

### Step 7: Test with Simple Pattern
- Draw a square
- Draw a circle
- Draw a simple face outline
- Verify NO skipped commands

### Step 8: Tune Timing
- Adjust `MIN_COMMAND_INTERVAL` if needed
- Adjust speeds if needed
- Goal: 100% reliability, don't care about speed

---

## Phase 6: Success Criteria

### Must Have
- ✅ Zero skipped commands
- ✅ Zero broken pens
- ✅ Pen reaches every point in order
- ✅ Smooth pen up/down transitions
- ✅ Completes full drawing without errors

### Nice to Have (Later)
- Fast drawing (optimize AFTER it works)
- Path optimization (AFTER reliability)
- Multiple image formats (AFTER core works)

---

## What We're Removing vs Keeping

### ❌ REMOVE
- G-code export/execution
- NGC/Inkscape compatibility
- PyBullet simulation
- Trajectory export (JSON/CSV)
- Douglas-Peucker simplification
- Path smoothing
- 2-opt optimization
- Adaptive feedrates
- Distance-based Z compensation
- Linear interpolation
- Force feedback
- Complex calibration routines
- Home position recording (maybe keep if used)

### ✅ KEEP
- Direct robot control via PyMyCobot
- Image capture
- Sketch generation (edge detection)
- Basic contour extraction
- Simple nearest-neighbor ordering
- Pen up/down control
- Movement synchronization (FIXED)
- Safety bounds checking
- Basic workspace validation

---

## Key Principles

1. **WAIT AFTER EVERY COMMAND** - No exceptions
2. **ONE THING AT A TIME** - No parallel operations
3. **SLOW IS RELIABLE** - Speed comes later
4. **FIXED Z HEIGHTS** - No compensation during drawing
5. **SIMPLE BEATS CLEVER** - Remove optimization until it works
6. **LOG EVERYTHING** - Know what command failed

---

## Expected Outcome

After this simplification:
- **~500 lines of code** instead of 2000+
- **One clear path**: Image → Sketch → Draw
- **Reliable execution**: Every point drawn, no skips
- **Maintainable**: Easy to debug and understand
- **Foundation for improvement**: Add features back ONE AT A TIME after core works

---

## Next Steps

1. Review this plan
2. Confirm you want to proceed
3. Backup current code
4. Start Phase 2 (delete unnecessary code)
5. Test with simple square drawing
6. Iterate on timing until 100% reliable
7. THEN add features back one at a time

**Remember**: A simple system that works is infinitely better than a complex system that's unreliable.
