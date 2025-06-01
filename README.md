# Image Detection Bot

A powerful automation tool that uses image detection to perform sequences of actions. Built with Python and PyQt6, this application allows you to create and manage sequences of image-based actions for automation tasks.

## Features

- **Image Detection**: Find and interact with images on screen
- **Sequence Editor**: Create and manage sequences of actions
- **Template Management**: Capture and manage image templates
- **Mouse Movement Control**: 
  - Curved mouse movement
  - Speed variation
  - Customizable control points
- **Failsafe System**: Jump to specific steps when certain images are detected
- **Loop Support**: Run sequences multiple times
- **Search Region**: Limit image detection to specific screen areas
- **Configuration Management**: Save and load different configurations

## Requirements

- Python 3.8 or higher
- PyQt6
- OpenCV
- PyAutoGUI
- NumPy
- Pillow
- darkdetect

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/image_detection_bot.git
cd image_detection_bot
```

2. Install required packages:
```bash
pip install -r requirements.txt
```

## Usage

1. **Starting the Application**
```bash
python bot_gui2.py
```

2. **Creating Templates**
   - Go to the Templates tab
   - Click "Add" to create a new template
   - Choose between capturing from screen or selecting an image file
   - Name your template and save it

3. **Creating Sequences**
   - Go to the Sequence tab
   - Click "Add" to create a new sequence
   - Add steps to your sequence:
     - Select a template to find
     - Configure search region, confidence, and timeout
     - Add actions (click, move, type, etc.)
   - Configure loop settings if needed
   - Set up failsafe conditions if desired

4. **Mouse Movement Settings**
   - Go to the Mouse Movement tab
   - Configure global mouse movement settings:
     - Enable/disable curved movement
     - Set control points
     - Adjust speed variation
     - Set steps per second

5. **Running Sequences**
   - Select a sequence from the list
   - Click "Run" to start execution
   - Use "Stop" to halt execution
   - Press F8 for emergency stop

## Configuration

The application saves configurations in JSON format, including:
- Templates and their image paths
- Sequences and their steps
- Global settings
- Search regions

## Mouse Movement Settings Explained

The mouse movement settings allow you to customize how the bot moves the mouse cursor. These settings can be configured globally in the Mouse Movement tab or per-action in the sequence editor.

### Curved Movement
- **Enabled**: Mouse follows a curved path instead of a straight line
- **Disabled**: Mouse moves in a straight line to the target
- **Effect**: More natural-looking movement that mimics human behavior

### Control Points
- **Minimum Control Point** (0.0 - 1.0)
  - Lower values (e.g., 0.1): Curve starts closer to the starting point
  - Higher values (e.g., 0.4): Curve starts further from the starting point
  - Default: 0.2
- **Maximum Control Point** (0.0 - 1.0)
  - Lower values (e.g., 0.6): Curve ends closer to the target
  - Higher values (e.g., 0.9): Curve ends further from the target
  - Default: 0.8
- **Effect**: Controls the shape of the curved path
  - Closer control points = tighter curve
  - Further control points = wider curve

### Speed Variation
- **Range**: 0.0 - 1.0
- **0.0**: Constant speed throughout the movement
- **0.5**: Moderate speed variation
- **1.0**: Maximum speed variation
- **Effect**: 
  - Lower values: More consistent, robotic movement
  - Higher values: More natural, human-like movement with acceleration/deceleration

### Steps per Second
- **Range**: 10 - 120
- **Lower values** (e.g., 20):
  - Fewer points in the movement path
  - More noticeable "steps"
  - Faster overall movement
- **Higher values** (e.g., 60):
  - More points in the movement path
  - Smoother movement
  - Slightly slower overall movement
- **Default**: 60
- **Effect**: Controls the smoothness vs. speed trade-off

### Example Configurations

1. **Natural Movement**:
   - Curved Movement: Enabled
   - Min Control: 0.2
   - Max Control: 0.8
   - Speed Variation: 0.7
   - Steps per Second: 60

2. **Fast and Direct**:
   - Curved Movement: Disabled
   - Speed Variation: 0.0
   - Steps per Second: 30

3. **Very Natural**:
   - Curved Movement: Enabled
   - Min Control: 0.3
   - Max Control: 0.7
   - Speed Variation: 0.9
   - Steps per Second: 90

## Tips

- Use the search region feature to limit image detection to specific areas
- Enable curved mouse movement for more natural-looking automation
- Set up failsafes to handle unexpected situations
- Use the confidence setting to adjust template matching sensitivity
- Save your configurations regularly

## Troubleshooting

- If image detection is not working:
  - Check template quality and lighting conditions
  - Adjust confidence threshold
  - Verify search region settings
- If mouse movement is erratic:
  - Adjust speed variation
  - Modify control points
  - Check steps per second setting

## Contributing

Fork your own version to modify please!


