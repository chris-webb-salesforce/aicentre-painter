# AI Centre Painter

A modular robot drawing system for MyCobot 320 with support for both direct drawing and ElephantRobotics workflow integration.

## Quick Start

```bash
python main_app.py
```

## Features

- 🎨 **Direct Drawing**: Convert camera images to robot drawings
- 📄 **G-code Support**: Export and execute G-code files 
- 🔧 **ElephantRobotics Integration**: Compatible with Inkscape plugin and RoboFlow
- 📍 **Home Position Recording**: Following official ElephantRobotics workflow
- 🎯 **Workspace Calibration**: Coordinate validation and testing
- 🖼️ **Image Processing**: Face detection and sketch conversion

## Project Structure

```
aicentre-painter/
├── main_app.py              # 🚀 Main application - START HERE
├── config.py                # ⚙️ Configuration settings
├── robot_controller.py      # 🤖 Robot control and movement
├── gcode_handler.py         # 📄 G-code/NGC file operations
├── image_processor.py       # 🖼️ Image processing and sketching
├── workspace_manager.py     # 📍 Workspace & home position management
├── archive/                 # 📦 Old files (for reference only)
└── README.md               # 📖 This file
```

## Workflows

### 1. Direct Drawing (Image → Robot)
1. Run `python main_app.py`
2. Select option 1: "Create new drawing from image"
3. Create sketch from captured image
4. Choose drawing method (direct, G-code, or NGC export)

### 2. ElephantRobotics Workflow (Inkscape → Robot)
1. Record home position (option 4 in main menu)
2. Create NGC file in Inkscape with ElephantRobotics plugin
3. Execute NGC file (option 3 in main menu)

### 3. G-code Workflow
1. Export G-code from image processing
2. Execute G-code file (option 2 in main menu)

## Configuration

All settings are in `config.py`:

- **Workspace dimensions**: `DRAWING_AREA_WIDTH_MM`, `DRAWING_AREA_HEIGHT_MM`
- **Robot settings**: `SERIAL_PORT`, `DRAWING_SPEED`, `PEN_DRAWING_Z`
- **Image processing**: `CONTOUR_SIMPLIFICATION_FACTOR`, `MIN_CONTOUR_AREA`

## Dependencies

```bash
pip install pymycobot opencv-python numpy
```

## Hardware Setup

1. MyCobot 320 robot arm
2. Pen holder attachment
3. Drawing surface (horizontal table setup)
4. USB camera (optional, for image capture)

## Archive

The `archive/` folder contains old implementations and debug files:
- `horizontal_table_drawing.py` - Original monolithic implementation
- Various test and debug scripts
- Alternative implementations

These are kept for reference but not needed for normal operation.

## Support

For issues or questions:
1. Check configuration in `config.py`
2. Test coordinate system (option in main menu)
3. Verify robot connection and home position
4. Review archive files for reference implementations