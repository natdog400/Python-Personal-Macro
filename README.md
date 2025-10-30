# Image Detection Bot

A powerful automation tool that uses image detection to perform sequences of actions. Built with Python and PyQt6, this application allows you to create and manage sequences of image-based actions for automation tasks.

## Features

- **Image Detection**: Find and interact with images on screen
- **Sequence Editor**: Create/copy/delete/rename and manage sequences of actions
- **Template Management**: Capture and manage image templates
- **Mouse Control**: 
  - Precise mouse movement and clicking
  - Support for left/right/middle mouse buttons
  - Configurable click intervals
- **Failsafe System**: Jump to specific steps when certain images are detected
- **Loop Support**: Run sequences multiple times
- **Search Region**: Limit image detection to specific screen areas
- **Configuration Management**: Save and load different configurations
- **Error Handling**: Comprehensive logging and error recovery

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

## Mouse Control

The bot provides precise control over mouse movements and clicks with the following features:

### Movement
- **Linear Movement**: Direct path to target coordinates
- **Random Region Targeting**: Option to click within a specified region
- **Speed Control**: Configurable movement duration

### Click Actions
- **Button Support**: Left, Right, and Middle mouse buttons
- **Multiple Clicks**: Support for single or multiple clicks
- **Click Intervals**: Configurable delay between multiple clicks
- **Click Verification**: Confirms successful click execution

### Error Handling
- **Position Validation**: Ensures target coordinates are within screen bounds
- **Movement Verification**: Confirms mouse reached target position
- **Detailed Logging**: Comprehensive error messages for troubleshooting

## Tips

- Use the search region feature to limit image detection to specific areas
- Adjust the confidence threshold for template matching to balance between accuracy and speed
- Use the random region feature for more natural clicking behavior
- Set up failsafes to handle unexpected situations
- Save your configurations regularly
- Check the logs for detailed information about bot operations

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

Fork your own version to modify please! Everything listed may not implemeted or is WIP!!


