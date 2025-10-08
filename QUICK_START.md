# Quick Start Guide - Drawing Within Bounds

## Understanding the Coordinate System

Your image is automatically scaled to fit your drawing area:

```
IMAGE (pixels)          →    PHYSICAL WORKSPACE (mm)
┌─────────────────┐          ┌─────────────────────────┐
│  400 x 600 px   │    →     │ ORIGIN_X to ORIGIN_X+120│
│                 │          │ ORIGIN_Y to ORIGIN_Y+180│
│  Full image     │          │                         │
│  area           │          │ Your drawing area       │
└─────────────────┘          └─────────────────────────┘
```

## Current Settings (from config.py)

```python
ORIGIN_X = 200.0          # Starting X position (mm)
ORIGIN_Y = 0.0            # Starting Y position (mm)
ORIGIN_Z = 10.0           # Table height (mm)

DRAWING_AREA_WIDTH_MM = 120   # 120mm wide
DRAWING_AREA_HEIGHT_MM = 180  # 180mm tall

# Your physical drawing area is:
# X: 200.0 to 320.0 mm
# Y: 0.0 to 180.0 mm
# Z: 10.0 mm (table surface)
```

## How to Verify Bounds

### Method 1: Test Drawing Area (Recommended)

1. Run the application:
   ```bash
   python3 main_app.py
   ```

2. Select option **2. Test drawing area**

3. This will draw a rectangle **20mm inside** your defined bounds:
   ```
   Test rectangle corners:
   - (220, 20) to (300, 160)  # 20mm margin on all sides
   ```

4. **Check:**
   - Does the pen stay within your paper?
   - Does it reach all corners without hitting limits?
   - Is the rectangle centered on your paper?

### Method 2: Visual Preview (Before Drawing)

When you create a drawing (option 1), you'll see a **preview window** showing:
- All contours numbered
- Start points (black dots)
- Travel paths (arrows)

**This preview shows EXACTLY what will be drawn** - if it looks good in preview, it will fit on the robot.

### Method 3: Trajectory Validation (Automatic)

The system automatically validates all points before drawing:

```
--- TRAJECTORY VALIDATION ---
✓ Total trajectory points: 342
✓ Actual bounds: X=[205.3, 315.7] Y=[5.2, 175.8]
✓ Safe workspace: X=[190.0, 330.0] Y=[-10.0, 190.0]
✅ All points within safe workspace bounds
```

If ANY point is outside bounds, you'll see:
```
❌ WARNING: 5 points out of bounds!
⚠️  Out of bounds: Contour 12, Point 3: (325.5, 185.2)
```

## Adjusting the Drawing Area

### If Drawing Area is Too Large

Your robot can't reach the full area, **reduce the size**:

**Edit `config.py`:**
```python
# Make it smaller
DRAWING_AREA_WIDTH_MM = 100   # Was 120
DRAWING_AREA_HEIGHT_MM = 150  # Was 180
```

### If Drawing Area is Too Small

You have more space available, **increase the size**:

**Edit `config.py`:**
```python
# Make it larger
DRAWING_AREA_WIDTH_MM = 150   # Was 120
DRAWING_AREA_HEIGHT_MM = 200  # Was 180
```

### If Drawing is Offset (Wrong Position)

The drawing is in the wrong place on your table, **adjust the origin**:

**Edit `config.py`:**
```python
# Move drawing area to the right
ORIGIN_X = 250.0  # Was 200.0

# Move drawing area forward
ORIGIN_Y = 50.0   # Was 0.0
```

## Finding Your Optimal Settings

### Step 1: Measure Your Physical Setup

1. **Place paper** on drawing surface
2. **Measure paper size** (e.g., 100mm x 150mm)
3. **Note where** you want the origin (top-left corner of paper)

### Step 2: Find Origin Position

**Method A: Use Manual Control (Recommended)**

1. Use RoboFlow or manual jogging to move pen to **top-left corner** of paper
2. Get current position:
   ```python
   python3 -c "from pymycobot import MyCobot320; mc = MyCobot320('/dev/ttyAMA0', 115200); print(mc.get_coords())"
   ```
3. Note the X, Y, Z values - these are your `ORIGIN_X`, `ORIGIN_Y`, `ORIGIN_Z`

**Method B: Start with Defaults and Adjust**

1. Run **option 2** (Test drawing area)
2. Watch where the rectangle is drawn
3. Adjust `ORIGIN_X` and `ORIGIN_Y` in small increments (±10mm)
4. Re-test until rectangle matches your paper

### Step 3: Set Drawing Area Size

**Edit `config.py`:**
```python
# Use your measured paper size (minus small margin)
DRAWING_AREA_WIDTH_MM = <your_paper_width> - 10
DRAWING_AREA_HEIGHT_MM = <your_paper_height> - 10
```

The 10mm margin ensures you don't draw right to the edge.

### Step 4: Verify with Test Rectangle

Run **option 2** again - the rectangle should now:
- ✅ Fit perfectly on your paper
- ✅ Stay within robot's reach
- ✅ Have even margins

## Example Configurations

### Small Paper (A6 - 105x148mm)
```python
ORIGIN_X = 200.0
ORIGIN_Y = 0.0
ORIGIN_Z = 10.0
DRAWING_AREA_WIDTH_MM = 95   # 105mm - 10mm margin
DRAWING_AREA_HEIGHT_MM = 138  # 148mm - 10mm margin
```

### Medium Paper (A5 - 148x210mm)
```python
ORIGIN_X = 180.0
ORIGIN_Y = -20.0
ORIGIN_Z = 10.0
DRAWING_AREA_WIDTH_MM = 138  # 148mm - 10mm margin
DRAWING_AREA_HEIGHT_MM = 200  # 210mm - 10mm margin
```

### Large Paper (A4 - 210x297mm)
```python
ORIGIN_X = 150.0
ORIGIN_Y = -50.0
ORIGIN_Z = 10.0
DRAWING_AREA_WIDTH_MM = 200  # 210mm - 10mm margin
DRAWING_AREA_HEIGHT_MM = 287  # 297mm - 10mm margin
```

## Image Aspect Ratio

The image processor uses a **400x600 pixel** (2:3 ratio) image.

**Important:** This maps to your `DRAWING_AREA_WIDTH_MM` x `DRAWING_AREA_HEIGHT_MM`:
- If your drawing area is 120x180mm (also 2:3 ratio) → **Perfect, no distortion**
- If your drawing area is 100x100mm (1:1 ratio) → **Image will be stretched**

### To Maintain Aspect Ratio

**Option 1: Match Image Ratio to Paper**

If using square paper (100x100mm):
```python
# Edit config.py
IMAGE_WIDTH_PX = 400
IMAGE_HEIGHT_PX = 400  # Was 600 - now square like paper
```

**Option 2: Match Paper Ratio to Image**

Keep 2:3 image ratio, make paper match:
```python
DRAWING_AREA_WIDTH_MM = 120
DRAWING_AREA_HEIGHT_MM = 180  # 2:3 ratio
```

## Troubleshooting

### "❌ WARNING: Points out of bounds!"

**Cause:** Drawing area defined in config is larger than robot can reach

**Fix:**
1. Reduce `DRAWING_AREA_WIDTH_MM` and `DRAWING_AREA_HEIGHT_MM`
2. OR move `ORIGIN_X` and `ORIGIN_Y` closer to robot base

### Drawing is Cut Off

**Cause:** Origin position is wrong OR drawing area too large

**Fix:**
1. Run test rectangle (option 2)
2. Verify rectangle stays on paper
3. Adjust origin or size in `config.py`

### Drawing is Tiny

**Cause:** Drawing area is very small, image gets scaled down

**Fix:**
1. Increase `DRAWING_AREA_WIDTH_MM` and `DRAWING_AREA_HEIGHT_MM`
2. Verify robot can reach the full area
3. Run test rectangle to confirm

### Drawing is in Wrong Location

**Cause:** Origin is not at top-left corner of your paper

**Fix:**
1. Manually jog robot to where you want top-left corner
2. Read position: `mc.get_coords()`
3. Update `ORIGIN_X` and `ORIGIN_Y` in `config.py`

## Safety Checks (Automatic)

The system has **three layers** of bounds checking:

1. **Trajectory Validation** - Before drawing starts
   - Checks all points against workspace bounds
   - Warns if any points are outside

2. **Z-Coordinate Validation** - During movement
   - Clamps Z to safe range: `MIN_SAFE_Z` to `MAX_SAFE_Z`
   - Prevents pen from going too low (breaking) or too high

3. **Workspace Bounds** - Configuration limits
   - 10mm safety margin added to drawing area
   - Points outside this trigger warnings

## Quick Calibration Checklist

- [ ] Measure paper size
- [ ] Find origin position (top-left corner of paper)
- [ ] Update `ORIGIN_X`, `ORIGIN_Y`, `ORIGIN_Z` in `config.py`
- [ ] Update `DRAWING_AREA_WIDTH_MM`, `DRAWING_AREA_HEIGHT_MM` in `config.py`
- [ ] Run **option 2** (Test drawing area)
- [ ] Verify test rectangle fits on paper
- [ ] Calibrate pen pressure (**option 3**)
- [ ] Try a simple drawing (**option 1**)
- [ ] Adjust settings if needed

---

**Remember:** The system **automatically scales** your image to fit the defined drawing area. As long as you configure the drawing area correctly, all images will stay within bounds!
