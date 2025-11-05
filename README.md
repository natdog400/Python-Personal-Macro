# Image Detection Bot

Automate mouse/keyboard actions by detecting on-screen images. This repo ships a PyQt6 GUI to manage templates, create sequences of actions, and configure a visual failsafe that can interrupt or be tested on demand. Built for personal automation, UI testing, and tinkering.

## Features

- **GUI Tabs**: Sequences, Failsafe, Templates, and Template Tester
- **Template Management**: Create, preview, capture from screen, or load from file
- **Sequence Editor**: Add steps that find a template and then run actions
- **Action Types**: `click`, `right_click`, `double_click`, `move`, `move_to`, `type`, `key_press`, `wait`, `scroll`, `click_and_hold`
- **Regions & Randomization**:
  - Step-level `search_region` for finding templates
  - Click actions can target a random point in a selected region(REMOVED not needed)
  - Move-To actions can use a selected region with optional random movement
- **Failsafe System**: Enable a template-based trigger, define a separate sequence, and test it with a button
- **Template Tester**: Live preview of template matching with confidence meter and optional search region
- **Hotkeys & Status**: F8 to stop; status bar updates; mouse position tracker
- **Config Persistence**: Reads/writes `config.json` and keeps template paths relative when possible
- **Break Settings**: Set a maximum runtime cap (hours/minutes/seconds); live timer shows `Elapsed / Max`; cap overrides loop settings

## Requirements

- Python 3.8+ (Windows batch launcher checks 3.8+)
- `opencv-python`, `pyautogui`, `numpy`, `Pillow`, `PyQt6`, `pyqt6-tools`, `darkdetect`
- Install via `requirements.txt`

## Installation

- Clone or download this repository
- Install dependencies:
  ```
  pip install -r requirements.txt
  ```
- Launch the GUI:
  - Cross-platform:
    ```
    python bot_gui.py
    ```
  - Windows convenience launcher:
    ```
    launch_gui.bat
    ```

## Quick Start

- Templates tab:
  - Click Add Template, set a name and image path
  - Use Capture to grab from screen, or Load Image to pick a file
  - Preview auto-updates
- Sequences tab:
  - Add a sequence, then add steps
  - For each step: set `find` template, `required`, `timeout`, optional `confidence`
  - Add actions like Click, Move, Move-To, Type, Key Press, Wait, Scroll
  - For Click: (optionally select a region to click randomly inside)REMOVED not needed)
  - For Move-To: optionally enable random and select a region to move within
  - Run the selected sequence; press F8 to stop
- Template Tester tab:
  - Pick a template, optionally select region, start live preview to see confidence value
- Failsafe tab:
  - Enable failsafe, choose a template and confidence, optionally set a search region
  - Build a separate “failsafe sequence” of steps
  - Click Test Failsafe to run only the failsafe sequence

## Configuration File

The app reads/writes `config.json` in the script directory. Template paths are converted to relative paths when possible.

Top-level structure:

```json
{
  "templates": {
    "TemplateName": "images/Template.png"
  },
  "sequences": [
    {
      "name": "Example",
      "steps": [
        {
          "find": "TemplateName",
          "required": true,
          "confidence": 0.8,
          "timeout": 10,
          "search_region": [x, y, width, height],
          "actions": [
            {"type": "move_to", "duration": 0.5},
            {"type": "click"},
            {"type": "type", "text": "hello"},
            {"type": "key_press", "key": "enter"},
            {"type": "wait", "seconds": 0.5},
            {"type": "scroll", "pixels": -300},
            {"type": "click_and_hold", "duration": 1.0}
          ]
        }
      ],
      "loop": false,
      "loop_count": 1
    }
  ],
  "failsafe": {
    "enabled": true,
    "template": "TemplateName",
    "confidence": 0.8,
    "region": [x, y, width, height],
    "sequence": [
      {
        "find": "AnotherTemplate",
        "required": true,
        "timeout": 10,
        "actions": [{"type": "click"}]
      }
    ]
  },
  "break_settings": {
    "enabled": true,
    "max_runtime_seconds": 3600
  }
}
```

Action dictionary fields (as used across sequences and failsafe steps):

- Common: `type`
- Mouse actions:
  - `button` (`left`/`right`/`middle`), `clicks` (int)
  - `x`, `y` (for absolute `move`), `duration`
  - `region` (for click randomization), `random`, `random_region` (for random move-to)
- Keyboard: `text` (type), `key` (key_press)
- Timing/scroll: `seconds` (wait), `pixels` (scroll)

## How Matching and Actions Work

- Template matching uses OpenCV (`cv2.matchTemplate`). Confidence threshold is adjustable per step.
- If a step has no `find` template, its actions run directly.
- `move_to` can target the detected position or a random point inside a selected region.
- `click` defaults to current mouse location unless `force_move` is used internally for region clicks.
- `click_and_hold` acts at the bot’s current position—usually set by a prior `move`/`move_to`.

## Failsafe Behavior

- The worker periodically checks the configured failsafe template (`check_failsafe_trigger`).
- When detected, the failsafe sequence executes (`execute_failsafe_sequence`).
- The Failsafe tab’s Test button runs only the failsafe sequence, using the current GUI configuration.

## Break Settings

- Use the Break Settings tab to set a maximum runtime using `hours`, `minutes`, and `seconds`.
- When enabled, the cap applies to total runtime and overrides any sequence loop count.
- The status bar shows a live clock: `Elapsed: Hh Mm Ss / Max: Hh Mm Ss`.
- The bot stops automatically when elapsed ≥ max runtime.
- These values persist to `config.json`:
  - `break_settings.enabled`: boolean
  - `break_settings.max_runtime_seconds`: integer seconds
- Tips:
  - Verify with a small cap (e.g., `0h 0m 10s`).
  - Ensure `Enable Max Runtime` is checked; a cap of `0` disables enforcement.
  - Check `bot_debug.log` for entries like `Max runtime reached (...)`.

## Logging & Debugging

- Runtime logs: `bot_debug.log`
- On failed matches, screenshots and templates may be saved under a `debug/` folder next to the script
- Press F8 to stop sequences; the status bar shows progress and messages

## Tips for Reliable Matching

- Use small, high-contrast templates of the exact UI you want to detect
- Avoid scale/rotation changes; match works best for identical sizes
- Consider using `search_region` to narrow detection for speed and accuracy
- For random click/move regions, ensure coordinates are on-screen and correct

## Known Limitations

- Template matching is sensitive to scaling and rotation
- Some GUI interactions are evolving; if you hit issues (e.g., editing failsafe steps), check logs and report
- Not all advanced scenarios are fully implemented; contributions are welcome

## Contributing

- Issues and PRs are appreciated
- Keep changes focused and documented
- Please avoid using this tool for anything that breaks app/game ToS

## License

This project is for personal use; choose an appropriate license before public release if needed
