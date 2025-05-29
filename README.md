# Image Detection Bot

Please help and contribute if you can find bugs im not the best coder and ai was used in the aid of creating this python script enjoy

A Python-based bot with GUI for detecting images on screen and automating sequences of actions. Ideal for automating repetitive tasks, game automation, and UI testing with visual feedback.

## Features

- **Crappy GUI**: an interface for managing templates and sequences
- **Template Management**: Create and manage image templates with previews
- **Sequence Editor**: Visually create and edit action sequences
- **Multiple Action Types**: Supports clicks, right-clicks, double-clicks, mouse movements, typing, and key presses
- **Template Matching**: Advanced image detection with adjustable confidence levels
- **Screen Region Selection**: Define specific screen regions for image detection
- **Looping Support**: Run sequences multiple times with configurable loop counts
- **Real-time Status**: Monitor bot activity with detailed status updates
- **Global Hotkey**: Stop sequences at any time with the F8 key
- **Failsafe Image step jump**: Will search for this image after every step and if found jump to step

## Requirements

- Python 3.9+
- OpenCV (for image processing)
- PyAutoGUI (for mouse and keyboard control)
- NumPy (for numerical operations)
- Pillow (for image handling)
- PyQt6 (for the GUI)
- darkdetect (for automatic dark/light theme)

## Installation

1. Clone this repository or download the files
2. Install the required packages:
   ```
   pip install -r requirements.txt
   ```
3. Launch the application:
   ```
   python bot_gui.py
   ```
   Or use the provided batch file on Windows:
   ```
   launch_gui.bat
   ```

## Getting Started

1. **Create Templates**:
   - Click "Add Template" to create a new template
   - Capture from screen or select an image file
   - Give your template a descriptive name

2. **Create Sequences**:
   - Click "Add Sequence" to create a new sequence
   - Add steps to your sequence
   - For each step, select a template and define actions
   - Set timeouts and required flags as needed

3. **Run Sequences**:
   - Select a sequence from the list
   - Click "Run" to start the sequence
   - Press F8 at any time to stop the sequence

## Configuration

The application automatically saves your configuration to `config.json` in the application directory. You can also import/export configurations using the File menu.

## Advanced Features

### Template Options
- **Confidence Threshold**: Adjust how strictly templates must match
- **Search Region**: Limit template search to specific screen areas
- **Required/Optional**: Mark steps as required or optional

### Action Types
- **Click**: Left/right/middle click with configurable number of clicks
- **Move**: Move mouse to coordinates or template location
- **Type**: Simulate keyboard typing
- **Key Press**: Simulate single key presses
- **Wait**: Pause for a specified duration
- **Loop**: Repeat sequences a set number of times

### Hotkeys
- **F8**: Stop the currently running sequence
- **Ctrl+O**: Open configuration
- **Ctrl+S**: Save configuration
- **Ctrl+N**: New configuration

## Template Images

- Use clear, high-contrast images for best results
- The template image should be an exact match for what you want to find
- Avoid using templates that appear multiple times on screen unless you want to find all instances

## Notes

- The bot uses template matching which is sensitive to scale and rotation
- For better performance, crop your templates to the smallest possible region
- Performance may vary depending on screen resolution and system resources
- NOT EVERYTHING FULLY WORKS AS INTENDED OR IS 100% FULLY IMPLEMENTED
