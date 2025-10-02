# Drawing Quality Tuning Guide

Quick reference for adjusting drawing quality, smoothness, and speed.

---

## Movement Smoothness (Fix Jumpiness)

### Config Presets ([config.py](config.py#L52))

**Currently Active: PRESET 2 (Balanced)**

```python
# To change, uncomment a different preset:

# PRESET 1: RELIABLE - Slow but guaranteed (original)
# PRESET 2: BALANCED - Smooth and reliable ← CURRENT
# PRESET 3: SMOOTH - Fast and fluid
# PRESET 4: MAXIMUM SPEED - Experimental
```

### Speed Settings ([config.py](config.py#L47))

```python
TRAVEL_SPEED = 30    # Pen up movements
DRAWING_SPEED = 15   # Drawing lines ← KEY for smoothness
APPROACH_SPEED = 20  # Lowering pen
LIFT_SPEED = 25      # Raising pen
```

**To make smoother:** Increase `DRAWING_SPEED` by 5 (try 20, then 25)
**If skipping points:** Decrease `DRAWING_SPEED` by 5

### Advanced: Wait Strictness ([robot_controller.py](robot_controller.py#L61))

```python
required_stable_readings = 2  # Current: balanced

# Change to:
# 1 = fast/smooth (may skip on complex paths)
# 2 = balanced ← CURRENT
# 3 = strict (slower but guaranteed)
```

```python
if movement < 0.5:  # Current: loose tolerance

# Change to:
# 0.2 = strict (waits until nearly still)
# 0.5 = balanced ← CURRENT
# 1.0 = loose (moves on quickly)
```

---

## Line Quality (Detail and Smoothness)

### Detail Level ([image_processor.py](image_processor.py#L111))

Controls how much detail is captured from the image:

```python
11, 2   # DETAILED ← CURRENT (lots of contours)
15, 4   # Medium detail
21, 6   # Simple/bold lines only
```

**More detail** = Lower first number (try 9 or 7)
**Less detail** = Higher first number (try 15 or 21)

### Contour Simplification ([config.py](config.py#L95))

Controls how many points are kept per contour:

```python
CONTOUR_SIMPLIFICATION_FACTOR = 0.003  # Very detailed

# Try:
# 0.001 = extremely detailed (many points, slower)
# 0.003 = detailed ← CURRENT
# 0.01  = simplified (fewer points, faster)
```

**Smoother curves** = Lower value (more points)
**Faster drawing** = Higher value (fewer points)

### Contour Smoothing ([config.py](config.py#L96))

Controls how smooth the lines are:

```python
CONTOUR_SMOOTHING = 3  # Medium smooth ← CURRENT

# Try:
# 0 = No smoothing (jagged/original)
# 2 = Light smoothing (subtle)
# 3 = Medium smoothing ← CURRENT
# 5 = Heavy smoothing (very rounded)
# 7 = Maximum smoothing (may lose detail)
```

**More natural curves** = Higher value
**Preserve sharp corners** = Lower value

### Min Contour Area ([config.py](config.py#L94))

Filters out tiny details:

```python
MIN_CONTOUR_AREA = 20  # Keep small details ← CURRENT

# Try:
# 10 = Keep tiny details (more contours, slower)
# 20 = Balanced ← CURRENT
# 50 = Skip small details (fewer contours, faster)
```

---

## Quick Tuning Recipes

### "My drawing is too simple/missing detail"

1. **Lower detail threshold**: Change `11, 2` → `9, 2` in [image_processor.py](image_processor.py#L111)
2. **Keep more points**: Change `CONTOUR_SIMPLIFICATION_FACTOR = 0.001` in [config.py](config.py#L95)
3. **Keep smaller contours**: Change `MIN_CONTOUR_AREA = 10` in [config.py](config.py#L94)

### "My lines are too jagged/angular"

1. **Increase smoothing**: Change `CONTOUR_SMOOTHING = 5` in [config.py](config.py#L96)
2. **More points**: Change `CONTOUR_SIMPLIFICATION_FACTOR = 0.001` in [config.py](config.py#L95)

### "My drawing is too slow"

1. **Use SMOOTH preset**: Uncomment PRESET 3 in [config.py](config.py#L66)
2. **Increase speeds**: Bump all speeds by 10 in [config.py](config.py#L47)
3. **Simplify more**: Change `CONTOUR_SIMPLIFICATION_FACTOR = 0.01` in [config.py](config.py#L95)

### "Robot is skipping points/unreliable"

1. **Use RELIABLE preset**: Uncomment PRESET 1 in [config.py](config.py#L54)
2. **Decrease speeds**: Lower all speeds by 5 in [config.py](config.py#L47)
3. **Stricter waiting**: Change `required_stable_readings = 3` in [robot_controller.py](robot_controller.py#L61)

### "Movement is jumpy/stop-start"

1. **Use BALANCED or SMOOTH preset**: [config.py](config.py#L60)
2. **Increase drawing speed**: Change `DRAWING_SPEED = 20` in [config.py](config.py#L48)
3. **Looser tolerance**: Change `movement < 1.0` in [robot_controller.py](robot_controller.py#L73)

---

## Recommended Starting Points

### For Portraits/Faces (Current Settings)
```python
# Detail
11, 2  # Detailed lines
CONTOUR_SIMPLIFICATION_FACTOR = 0.003
CONTOUR_SMOOTHING = 3
MIN_CONTOUR_AREA = 20

# Speed
PRESET 2: BALANCED
DRAWING_SPEED = 15
```

### For Simple Sketches/Bold Lines
```python
# Detail
15, 4  # Medium detail
CONTOUR_SIMPLIFICATION_FACTOR = 0.01
CONTOUR_SMOOTHING = 5
MIN_CONTOUR_AREA = 50

# Speed
PRESET 3: SMOOTH
DRAWING_SPEED = 20
```

### For Maximum Detail/Realistic
```python
# Detail
9, 2   # Very detailed
CONTOUR_SIMPLIFICATION_FACTOR = 0.001
CONTOUR_SMOOTHING = 2
MIN_CONTOUR_AREA = 10

# Speed
PRESET 1: RELIABLE
DRAWING_SPEED = 10
```

### For Speed/Performance Testing
```python
# Detail
21, 6  # Simple
CONTOUR_SIMPLIFICATION_FACTOR = 0.02
CONTOUR_SMOOTHING = 7
MIN_CONTOUR_AREA = 100

# Speed
PRESET 4: MAXIMUM SPEED
DRAWING_SPEED = 25
```

---

## Troubleshooting

### Problem: Drawing looks good in preview but bad on paper

**Cause:** Pen pressure or Z height issue

**Fix:**
1. Run **Option 3: Calibrate pen pressure**
2. Adjust `PEN_DRAWING_Z` in [config.py](config.py#L31)

### Problem: Some contours are missing

**Cause:** Filtered out as too small

**Fix:**
- Lower `MIN_CONTOUR_AREA` to 10 or even 5

### Problem: Lines are doubled

**Cause:** Detecting both sides of edges

**Fix:**
- Already using `RETR_EXTERNAL` (good)
- Try increasing detail threshold: `15, 4` instead of `11, 2`

### Problem: Pen breaks / goes too low

**Cause:** Z height too aggressive

**Fix:**
1. Increase `PEN_DRAWING_Z` (make it less negative) in [config.py](config.py#L31)
2. Use calibration tool (**Option 3**)

### Problem: Timeout errors during drawing

**Cause:** Movement taking too long

**Fix:**
1. Increase `MOVEMENT_TIMEOUT` in [config.py](config.py#L62)
2. OR decrease `DRAWING_SPEED` in [config.py](config.py#L48)

---

## Testing Workflow

1. **Start with BALANCED preset** (current default)
2. **Draw a test image** (simple face or shape)
3. **Observe:**
   - Is it smooth? ✅ Good
   - Jumpy? → Increase `DRAWING_SPEED` or use SMOOTH preset
   - Skipping points? → Use RELIABLE preset or decrease speeds
   - Missing detail? → Lower detail threshold or simplification
   - Too detailed? → Increase detail threshold or simplification

4. **Adjust one setting at a time**
5. **Test again**
6. **Repeat until optimal**

---

## Performance Comparison

| Preset | Points/sec | Smoothness | Reliability | Recommended For |
|--------|-----------|------------|-------------|-----------------|
| RELIABLE | ~2 | ★★☆☆☆ | ★★★★★ | First time setup |
| BALANCED | ~5 | ★★★★☆ | ★★★★☆ | General use ← **DEFAULT** |
| SMOOTH | ~10 | ★★★★★ | ★★★☆☆ | Simple sketches |
| MAX SPEED | ~20 | ★★★★★ | ★★☆☆☆ | Testing/demos |

---

**Remember:** You can always revert to safe defaults by uncommenting **PRESET 1: RELIABLE**!
