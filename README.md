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
  
   - 4. **Random Click in Region**:
   - When editing a step, add a "Click" action.
   - Use the "Select Region" button to define an area.
   - The bot will click at a random point inside this region each time the action runs.
  
   - 5. **Random Move-To in Region**:
   - When editing a step, add a "Move To" action.
   - Enable the "Toggle Random" checkbox to activate random movement.
   - Use the "Select Region" button to define an area.
   - The bot will move to a random point inside this region for the template step. If not enabled, it moves to the center of the detected template as usual.

## Configuration

The application automatically saves your configuration to `config.json` in the application directory. You can also import/export configurations using the File menu.

## Advanced Features

### Template Options
- **Confidence Threshold**: Adjust how strictly templates must match
- **Search Region**: Limit template search to specific screen areas
- **Required/Optional**: Mark steps as required or optional
- **Click Region**: For click actions, you can select a region. The click will occur at a random location within this region.
- **Move-To Region**: For move_to actions, you can enable random movement and select a region. The move will occur at a random location within this region if random is enabled.


### Action Types
- **Click**: Left/right/middle click with configurable number of clicks
- **NEW:** You can now select a region for a click action. If a region is set, the click will happen at a random point inside that region.
- **Move**: Move mouse to coordinates or template location
  - **NEW:** You can enable random movement for move_to actions. When enabled, the bot will move to a random point inside a selected region for the template, instead of the center.
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
![explorer_DLFHKKejYv](https://github.com/user-attachments/assets/6232d9a5-9198-4846-8cb2-10fe655fc240)
