# Image Detection Bot

Automate mouse/keyboard actions by detecting on-screen images. This repo ships a PyQt6 GUI to manage templates, create sequences of actions, and configure a visual failsafe that can interrupt or be tested on demand. Built for personal automation, UI testing, and tinkering.

## Features

- **GUI Tabs**: Sequences, Failsafe, Templates, and Template Tester
- **Template Management**: Create, preview, capture from screen, or load from file
- **Sequence Editor**: Add steps that find a template and then run actions
- **Action Types**: `click`, `right_click`, `double_click`, `move`, `move_to`, `type`, `key_press`, `wait`, `scroll`, `click_and_hold`
- **Regions & Randomization**:
  - Step-level `search_region` for finding templates
  - Click actions can target a random point in a selected region
  - Move-To actions can use a selected region with optional random movement
- **Failsafe System**: Enable a template-based trigger, define a separate sequence, and test it with a button
- **Template Tester**: Live preview of template matching with confidence meter and optional search region
- **Hotkeys & Status**: F8 to stop; status bar updates; mouse position tracker
- **Config Persistence**: Reads/writes `config.json` and keeps template paths relative when possible
- **Break Settings**: Set a maximum runtime cap (hours/minutes/seconds); live timer shows `Elapsed / Max`; cap overrides loop settings

## Architecture

- Desktop GUI (PyQt6): author templates, sequences, groups, failsafe; run and observe step progress.
- Web Server: lightweight HTTP server with editors and a live MJPEG preview; mirrors most GUI features.
- IPC Bridge: the web server writes commands to `ipc_command.json`; the GUI polls and acts (run/stop/reload/etc.).
- Launcher (Python/Tk + PowerShell): unified start/stop/open; can edit and save `server_config.json` before launch.
- Configuration: `config.json` stores templates, sequences, failsafe, groups, schedules, break settings.
- Assets: `images/`, `failsafe_images/`, `web/static/`.

## Recent Updates (Web + GUI)

- Configuration safety and startup stability
  - Guarded auto‑save during startup with a suppression flag to prevent writing an empty `config.json` while UI constructs.
  - Wrapped early calls like `toggle_failsafe_ui` so they don’t override the saved configuration before load completes.
  - Prevented template tab auto‑switch during config reloads; GUI now defaults to the Sequences tab at startup and after reload.

- Sequence editor robustness
  - Rebuilds a fresh `SequenceEditor` whenever selection changes to avoid stale state and disappearing steps.
  - Added a `_rebuilding_sequence_editor` guard so config mirrors don’t write empty steps during transitions.
  - Preserves existing steps if an editor snapshot returns empty.
  - Added `SequenceEditor.update_groups(...)` so Group Call dropdowns stay in sync with current group names.

- Groups management
  - Fixed a bug where deleted groups reappeared: prevented `update_config_from_ui` from writing stale `group_editor_widget` contents back if the group was removed; cleared editor references after deletion.

- Failsafe improvements
  - Web → GUI template sync: desktop now reads both `failsafe.template_name` and `failsafe.template` and preserves selection during combo refresh. Server writes both keys for full symmetry.
  - Web editor now supports adding normal steps (not only Group Calls) and “Use Preview as Failsafe Region”.
  - Action editor mirrors Sequences behavior: shows only relevant fields per action type (wait → seconds, move_to/drag → X/Y/duration/random/region, keypress → key/modifiers, scroll → pixels, click → button/clicks).
  - Random movement: web Failsafe editor saves `random` and `random_region` so GUI runs move‑to in random mode correctly.

- Web UX and auth
  - Added top‑nav links for a consistent flow between Dashboard, Sequences, Groups, Controls, Schedules, Templates, Failsafe.
  - “Remember token” checkbox added to all pages (Dashboard/Controls/Sequences/Groups/Templates/Failsafe/Schedules); tokens sync via `localStorage` and auto‑populate across pages.
  - Sequences editor: added a “Group” dropdown next to “Add Group Call”; loads groups before rendering; always displays the selected group even if not yet in the global list.

- Defaults and quality of life
  - New steps (GUI and Web) carry sensible defaults: `confidence`, `detection_strategy`, `step_loops`, `monitor`, and a default `actions` list.
  - Injects a global `search_region` into new steps (when present) for faster authoring.
  - Group selectors default to the first available group for quick adds.

## Requirements

- Python 3.8+ (Windows batch launcher checks 3.8+)
- `opencv-python`, `pyautogui`, `numpy`, `Pillow`, `PyQt6`, `pyqt6-tools`, `darkdetect`
- Install via `requirements.txt`

## Setup

- Install dependencies:
  ```
  pip install -r requirements.txt
  ```
- Configuration files:
  - `config.json` — main app configuration (created/updated by GUI/web server)
  - `server_config.json` — web server settings (`bind`, `port`, `token`, `mjpeg_fps`, `backup_retention`)
  - Backups — stored in `backup configs/` with retention (`backup_retention` default 20)

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

### Running the Web Server

- Start the server (requires the same Python environment):
  ```
  python web_server.py
  ```
- Open `http://localhost:8765/` in your browser.
- Auth: the server reads `server_config.json` for `token`; pages include a “Remember token” checkbox that persists your token across all editors via `localStorage`.

### Python Launcher

- Start the interactive launcher:
  ```
  python launcher.py
  ```
- Choose to launch the GUI, the Web Server, or both.
- If launching the Web Server, you can either reuse the last `server_config.json` or edit settings (bind, port, token, `mjpeg_fps`, `backup_retention`) in the launcher and save them.
- On Windows, you can alternatively use `launch.bat` (PowerShell wrapper) for a menu-driven experience.

### Launcher Details

- `launcher.py` (Tkinter GUI)
  - Options: Launch GUI, Launch Web Server, or Both
  - “Use last settings” or “Edit and save new settings” for the web server
  - Buttons: Start, Open Web Portal (opens `http://localhost:<port>/?token=...`), Stop GUI, Stop Web Server, Exit
  - Compiled behavior: launcher runs without a console; GUI (`ImageDetectionBot.exe`) and server (`WebServer.exe`) run with console windows for logs.
- Batch wrappers
  - `run_launcher.bat`: starts `launcher.py` (prefers `venv\Scripts\python.exe`, falls back to `python`)
  - `launch.bat`: invokes `launch.ps1` (menu-driven console)

### Building (PyInstaller)

You can build the whole project in one go using the provided spec:

```
pip install pyinstaller
pyinstaller ProjectBundle.spec
```

This creates a `dist/ProjectBundle/` folder containing:

- `Launcher.exe` — Tkinter launcher to start GUI and/or Web Server
- `ImageDetectionBot.exe` — the desktop GUI
- `WebServer.exe` — the HTTP server for the web editors

Static assets (web pages, images, failsafe images) and configs (`server_config.json`, `config.json`) are included in the bundle.

Notes:
- If PyInstaller misses dependencies for your environment, add them to `hiddenimports` in `ProjectBundle.spec`.
- For one‑file (`--onefile`) builds, prefer building each binary separately; multi‑exe onefile is not supported.

#### Single Binary with Console Logs (GUI)

If you want a single executable that shows the GUI and keeps a visible console for debug logs:

```
pyinstaller --onefile SingleBotConsole.spec
```

This outputs `dist/ImageDetectionBotConsole.exe`, which launches the GUI and shows a console window with runtime logs. Assets and configs are included. Use this when you prefer seeing logs directly without opening the log file.

## Quick Start

- Templates tab:
  - Click Add Template, set a name and image path
  - Use Capture to grab from screen, or Load Image to pick a file
  - Preview auto-updates
- Sequences tab:
  - Add a sequence, then add steps
  - For each step: set `find` template, `required`, `timeout`, optional `confidence`
  - Add actions like Click, Move, Move-To, Type, Key Press, Wait, Scroll
  - For Click: optionally select a region to click randomly inside
  - For Move-To: optionally enable random and select a region to move within
  - Run the selected sequence; press F8 to stop
- Template Tester tab:
  - Pick a template, optionally select region, start live preview to see confidence value
- Failsafe tab:
  - Enable failsafe, choose a template and confidence, optionally set a search region
  - Build a separate “failsafe sequence” of steps
  - Click Test Failsafe to run only the failsafe sequence

## Screenshots

Screenshots illustrating the GUI and Web editors are stored under `docs/screenshots/`.

Suggested filenames (drop your PNGs in that folder):

- GUI
  - `gui_sequences.png`: Sequences tab with steps and actions
  - `gui_failsafe.png`: Failsafe tab with settings and sequence
  - `gui_templates.png`: Templates tab with preview and actions
- Web
  - `web_dashboard.png`: Dashboard page with preview and nav
  - `web_sequences.png`: Sequences editor (steps + actions + preview)
  - `web_failsafe.png`: Failsafe editor (settings + steps + preview)
  - `web_groups.png`: Groups editor (list + steps + nested actions)
  - `web_schedules.png`: Schedules page (rows with Enabled/Sequence/Time)

Screenshots are embedded below; place PNGs in `docs/screenshots/` using the shown filenames.

### Screenshot Gallery

- GUI
  - Sequences
    - ![GUISequenceTab](./docs/screenshots/GUISequenceTab.png?raw=1)
  - Failsafe
    - ![GUIFailSafeTab](./docs/screenshots/GUIFailSafeTab.png?raw=1)
  - Templates
    - ![GUITemplateTab](./docs/screenshots/GUITemplateTab.png?raw=1)
  - Template Tester
    - ![GUITemplateTesterTab](./docs/screenshots/GUITemplateTesterTab.png?raw=1)
  - Groups
    - ![GUIGroupsTab](./docs/screenshots/GUIGroupsTab.png?raw=1)
  - Scheduled Sequences
    - ![GUIScheduledSequenceTab](./docs/screenshots/GUIScheduledSequenceTab.png?raw=1)
  - Break Settings
    - ![GUIBreaksettings](./docs/screenshots/GUIBreaksettings.png?raw=1)

- Web
  - Dashboard
    - ![WEBDashboard](./docs/screenshots/WEBDashboard.png?raw=1)
  - Sequence Editor
    - ![WEBSequenceEditor](./docs/screenshots/WEBSequenceEditor.png?raw=1)
    - ![WEBLoadedSequenceinWEB](./docs/screenshots/WEBLoadedSequenceinWEB.png?raw=1)
  - Groups Editor
    - ![WEBGroupsEditor](./docs/screenshots/WEBGroupsEditor.png?raw=1)
  - Templates Manager
    - ![WEBTemplatesManager](./docs/screenshots/WEBTemplatesManager.png?raw=1)
  - Failsafe Settings
    - ![WEBFailsafeSettings](./docs/screenshots/WEBFailsafeSettings.png?raw=1)
  - Run Controls (with Break Settings)
    - ![WEBRunControlswithBreakSettings](./docs/screenshots/WEBRunControlswithBreakSettings.png?raw=1)
  - Scheduled
    - ![WEBScheduled](./docs/screenshots/WEBScheduled.png?raw=1)
  - Select Region (Preview)
    - ![WebSelectRegion](./docs/screenshots/WebSelectRegion.png?raw=1)

## Web UI (Editors & Controls)

The project includes a lightweight web server with browser-based editors that mirror most GUI features. Open pages via `http://localhost:8765/static/...` and append `?token=YOURTOKEN` if auth is enabled.

- Pages
  - `Sequences` (`/static/sequences.html`): sequence list, per-step editor, live preview
  - `Failsafe` (`/static/failsafe.html`): failsafe settings and sequence editor, live preview
  - `Groups` (`/static/groups.html`): group management (sequence-like collections), nested actions, live preview
  - `Controls` (`/static/control.html`): run/stop, non-required-wait toggle, “Run Group”

- Fully implemented in web editors
  - Per-step header controls (Sequences): `find` template, `required`, `confidence`, `timeout`, `monitor`, `Step Loops`, `Detection Strategy`, `Min Inliers`, `Ratio`, `RANSAC`, `Select Search Region`
- Per-action controls (Sequences/Failsafe): `type`, `button`, `clicks`, `x/y`, `duration`, `random`, `seconds`, `pixels`, `key`, `modifiers`, `Select Region`, `Set Random Region`
  - “Add Action” palette with sensible defaults (click, right/double click, move/move_to/drag, type, wait, scroll, click_and_hold)
  - Group Call steps: dropdown picker and save in Sequences and Failsafe; adds `{ call_group: "GroupName" }` step
  - Groups editor: CRUD for groups; per-step header (`find`, `required`, `timeout`, save, reorder/delete); nested actions with same controls as sequences/failsafe; “Add Action” palette
  - Live Preview on all editors (Sequences/Failsafe/Groups) with monitor selection; click‑drag selection draws a box and maps to natural image coordinates
  - Preview “Size” dropdown (640/800/1024/1280) on all editors; region mapping remains accurate regardless of browser zoom or selected size

- Partially implemented / known gaps
  - Groups step header advanced fields (monitor, Step Loops, Detection Strategy, Min Inliers/Ratio/RANSAC) are not yet exposed
  - Insert‑at‑index for new steps is not implemented; new steps append to the end
  - Label polish and defaults can be tuned based on your workflow

- Web API endpoints (selected)
  - Sequences
    - `GET /api/sequences` → list names
    - `GET /api/sequences/:name` → sequence details
    - `PUT /api/sequences/:name` → update metadata (`loop`, `loop_count`, rename)
    - `POST /api/sequences/:name/steps` → append step
    - `PUT /api/sequences/:name/steps/:idx` → update step
    - `DELETE /api/sequences/:name/steps/:idx` → delete step
    - `POST /api/sequences/:name/steps/reorder` → move a step
    - `POST /api/sequences/:name/steps/:idx/actions` → append action
    - `PUT /api/sequences/:name/steps/:idx/actions/:aidx` → update action
    - `DELETE /api/sequences/:name/steps/:idx/actions/:aidx` → delete action
    - `POST /api/sequences/:name/steps/:idx/actions/reorder` → move an action
  - Failsafe
    - `GET /api/failsafe` → settings
    - `PUT /api/failsafe` → update settings
    - `GET /api/failsafe/sequence` → list steps
    - `POST /api/failsafe/sequence` → append step
    - `PUT /api/failsafe/sequence/:idx` → update step
    - `DELETE /api/failsafe/sequence/:idx` → delete step
    - `POST /api/failsafe/sequence/reorder` → move step
    - `POST /api/failsafe/sequence/:idx/actions` → append action
    - `PUT /api/failsafe/sequence/:idx/actions/:aidx` → update action
    - `DELETE /api/failsafe/sequence/:idx/actions/:aidx` → delete action
    - `POST /api/failsafe/sequence/:idx/actions/reorder` → move action
  - Groups
    - `GET /api/groups` → list group names
    - `GET /api/groups/:name` → group details (supports dict‑ and list‑based storage)
    - `POST /api/groups` → create
    - `PUT /api/groups/:name` → update (rename, steps, loop, loop_count)
    - `DELETE /api/groups/:name` → delete
    - `POST /api/groups/:name/steps` → append step
    - `PUT /api/groups/:name/steps/:idx` → update step
    - `DELETE /api/groups/:name/steps/:idx` → delete step
    - `POST /api/groups/:name/steps/reorder` → move step
    - `POST /api/groups/:name/steps/:idx/actions` → append action
    - `PUT /api/groups/:name/steps/:idx/actions/:aidx` → update action
    - `DELETE /api/groups/:name/steps/:idx/actions/:aidx` → delete action
    - `POST /api/groups/:name/steps/:idx/actions/reorder` → move action
  - Controls
    - `POST /api/run-options` → set non‑required‑wait (IPC to GUI)
    - `POST /api/run` → run sequence by name (IPC to GUI)
    - `POST /api/run-group` → create a temporary `__RunGroup__` sequence from a group and run it (IPC)
    - `POST /api/stop` → stop current run (IPC)
  - Monitors & stream
    - `GET /api/monitors` → JSON list of monitors (index, width, height)
    - `GET /stream.mjpeg?monitor=...&token=...` → MJPEG stream for previews

- Region selection accuracy
  - The web editors compute coordinates inside the actual image content box (`object-fit: contain`) rather than the element bounds. This keeps natural pixel coordinates stable across preview size changes and browser zoom.
  - The overlay box is drawn in the pane at `paneOffset + contentOffset + contentCoord`, so the visual selection always matches the drawn image.

Tips
- After web edits, the server writes `config.json`, creates time‑stamped backups in the `backup configs/` folder (e.g., `config.backup.176257XXXX.json`), prunes older ones according to `backup_retention` in `server_config.json`, and triggers an IPC reload so the desktop GUI reflects changes without restart.
- If you prefer GUI editing, you can mix and match; web and desktop stay in sync.

### Web–Desktop Sync Details

- Failsafe template name
  - Web saves `failsafe.template_name` and the server also writes `failsafe.template` for backward compatibility.
  - Desktop reads either key and preserves selection when repopulating the combo; preview updates immediately.
- Sequences and Failsafe actions
  - Random movement: set `random=true` and a `random_region` in the action; desktop honors random move_to and random region clicks.
  - Action editors only expose fields relevant to the chosen type; saves mirror exactly what the desktop expects.
- Group Call steps
  - Web Sequences and Failsafe editors both support Group Call steps; Sequences has a top‑level “Group” dropdown for quick adds.
  - Editors load groups before rendering and ensure step selections are visible even if the group name wasn’t in the list yet.

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
- Feature matching supports multiple detectors and safe fallbacks:
  - Detectors: `ORB` (fast), `AKAZE` (scale-robust), `SIFT` (strong features; requires `opencv-contrib-python`).
  - Pipeline: KNN + Lowe’s ratio → RANSAC homography → sanity check (area ratio) → center of detected polygon.
  - Fallbacks: If detector fails, automatically tries `AKAZE`, then `SIFT`. If all fail, a multi‑scale template match runs.
  - Provide `strategy: "feature"` and optional `min_inliers`, `ratio_thresh`, `ransac_thresh` per step.
- If a step has no `find` template, its actions run directly.
- `move_to` can target the detected position or a random point inside a selected region.
- `click` defaults to current mouse location unless `force_move` is used internally for region clicks.
- `click_and_hold` acts at the bot’s current position—usually set by a prior `move`/`move_to`.

### Actions and Inputs (GUI & Web)

- click: `button`, `clicks`; optional `random` + `random_region` for random click inside region
- move: absolute `x`, `y`, `duration`; fallback to detected position in a step when started via web (MOVE without `x,y` treated like MOVE_TO)
- move_to: detected template position or explicit `x`, `y` + `duration`; optional `random` + `random_region`
- type: `text`
- key_press: `key`, optional `modifiers`
- wait: `seconds`
- scroll: `pixels`
- click_and_hold: `duration` at current position

## Multi-Monitor & Regions

- Toolbar Monitor Selector:
  - Choose All Monitors or a specific screen; status bar shows the active region.
  - Capture dialogs (Templates tab) respect the selection.
- Per-Step Monitor Override:
  - In the Step Editor, set the step's Monitor; this constrains detection to that screen when no `search_region` is set.
  - `monitor` persists in `config.json` as `null`, `"ALL"`, or `[x, y, w, h]`.
- Region Selection:
  - Select Search Region opens an overlay; if a monitor is chosen, overlay is restricted to that screen.
  - Regions and monitors can be combined; region takes precedence over monitor.
- Runtime Capture:
  - Uses per-monitor `QScreen.grabWindow(...)` with `devicePixelRatio()` for DPI-aware capture.
  - For full desktop, frames are stitched from each monitor according to virtual desktop coordinates.
  - Fallbacks: `PIL.ImageGrab.grab(bbox=...)` or `pyautogui.screenshot(region=...)` when needed.

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
- Optional final sequence on break:
  - Toggle: `Run final sequence when time is hit` and choose the sequence from the dropdown.
  - Behavior: when max runtime is reached, the primary run ends and the selected final sequence is launched once. The final sequence ignores the runtime cap and runs to completion.
  - Live refresh: the dropdown updates immediately when you add, delete, rename, or duplicate sequences — no restart required.
  - Config fields:
    - `break_settings.run_final_after_break`: boolean
    - `break_settings.final_sequence_name`: string (sequence name) or `"(none)"`

## Scheduled Sequences

- Use the Scheduled Sequences tab to start specific sequences automatically at a given time each day.
- Each schedule row has:
  - `Enabled`: whether the schedule is active
  - `Sequence`: the sequence to run
  - `Time`: daily start time (`HH:mm`, 24-hour)
- Behavior toggles:
  - `Queue if busy`: if a run is already in progress at the scheduled time, starts the scheduled sequence as soon as the current run completes.
  - `Preempt if busy`: stops the current run immediately and starts the scheduled sequence.
  - `Resume previous`: when preempting, resumes the original run after the scheduled sequence finishes.
  - Queue and Preempt are mutually exclusive when saved; if Preempt is enabled, Queue is saved as off.
- Behavior:
  - At the scheduled time, the app starts the selected sequence once per day.
  - If another run is already in progress, the scheduled run is skipped for that day.
  - Scheduled runs ignore the max runtime cap (they run to completion unless stopped).
  - The scheduler checks every 30 seconds.
- Persistence:
  - `scheduled_sequences`: array of schedule objects persisted to `config.json`, each with:
    - `enabled`: boolean
    - `sequence_name`: string
    - `time`: `hh:mm AM/PM` (12-hour); loader accepts legacy `HH:mm`.
    - `queue_if_busy`: boolean
    - `preempt_if_busy`: boolean
    - `resume_previous`: boolean
    - `last_run_date`: `YYYY-MM-DD` (used to ensure only one run per day)
- Tips:
  - Make sure your sequence exists and is valid before scheduling.
  - You can add/remove schedule rows anytime; changes are saved with the configuration.
  - New schedule rows default to `Enabled = off` — toggle on to activate.
  - The status bar shows `Next: <Sequence> @ hh:mm AM/PM` and updates immediately when you edit schedule rows.

## Toolbar Shortcuts

- The top toolbar includes quick actions for configuration:
  - `New`: create a fresh configuration
  - `Open`: load an existing `config.json`
  - `Save`: write current configuration
  - `Save As`: save configuration to a new file
  - These mirror the File menu and make switching configs faster.

## Scheduler Notes

- The scheduler runs only while the app is open.
- Time parsing supports both `hh:mm AM/PM` and `HH:mm`.
- When a run is active at the scheduled time:
  - With `Queue if busy` on, the scheduled run starts right after the active run completes.
  - With `Preempt if busy` on, the current run stops and the scheduled run starts immediately;
    if `Resume previous` is on, the original run resumes automatically after the scheduled run completes.
  - With neither on, the scheduled run is skipped for that minute.
- Tips:
  - Verify with a small cap (e.g., `0h 0m 10s`).
  - Ensure `Enable Max Runtime` is checked; a cap of `0` disables enforcement.
  - Check `bot_debug.log` for entries like `Max runtime reached (...)`.

## Logging & Debugging

- Runtime logs: `bot_debug.log`
- On failed matches, screenshots and templates may be saved under a `debug/` folder next to the script
- Press F8 to stop sequences; the status bar shows progress and messages

### Template Tester (Preview)

- Strategy Dropdown: Default (template) or Feature (scale/rotation).
- Parameters: Min Inliers, Ratio, RANSAC thresholds.
- Visualizer controls:
  - Detector: choose `ORB`, `AKAZE`, or `SIFT` for the feature preview.
  - Show Keypoints: toggle overlay of scene keypoints for visual debugging (off by default for performance).
 - Capture Backend:
   - Options: `Auto (best)`, `MSS`, `QScreen`.
   - Auto prefers `MSS` when available, otherwise uses `QScreen`.
   - An availability indicator shows whether the selected backend is usable.
- Debug Panel shows:
  - Target screen index, geometry, local capture rect
  - Frame size and bytes-per-line, DPI ratio, pixmap state
  - Backend used (MSS/QScreen) and fallback path (PIL bbox, pyautogui region, stitched full desktop)
- Metrics: Inliers, Matches, Confidence, RANSAC reprojection error.

### Monitor Info

- Toolbar button opens a dialog listing monitors: index, geometry, and DPI ratio.
- Capture All stitches a preview from all monitors to validate layout.

## Tips for Reliable Matching

- Use small, high-contrast templates of the exact UI you want to detect
- Avoid scale/rotation changes; match works best for identical sizes
- Consider using `search_region` to narrow detection for speed and accuracy
- For multi-monitor setups, prefer per-step monitor selection or regions on the target screen.
- Use Feature strategy for rotated/scaled UI elements; increase `min_inliers` or adjust `ratio_thresh` when noisy.
- For random click/move regions, ensure coordinates are on-screen and correct

## Known Limitations

- Template matching is sensitive to scaling and rotation
- Feature matching adds overhead; tune thresholds for your scene.
- Some capture backends may behave differently under extreme DPI or exotic layouts; robust fallbacks are implemented.
- Some GUI interactions are evolving; if you hit issues (e.g., editing failsafe steps), check logs and report
- Not all advanced scenarios are fully implemented; contributions are welcome

## Contributing

- Issues and PRs are appreciated
- Keep changes focused and documented
- Please avoid using this tool for anything that breaks app/game ToS

## License

This project is for personal use; choose an appropriate license before public release if needed
- IPC and compiled mode
  - Web server writes `ipc_command.json` to the executable folder when compiled; GUI reads and deletes it after handling.
  - Web server serves static pages from `web/static`; compiled builds also fall back to `_internal/web/static`.
  - Launcher’s “Open Web Portal” targets `http://localhost:<port>/?token=...` to avoid `0.0.0.0` in browsers.
- Web-start runs
  - The GUI applies `break_settings.enabled` and `break_settings.max_runtime_seconds` from `config.json` when sequences are started via web IPC.
  - Status bar shows `Max runtime: Hh Mm Ss` and live elapsed.
  - On cap hit, the run stops and runs the configured final sequence once (if set).
## Troubleshooting

- Web portal 404 on root (`/?token=...`) in compiled build:
  - Launch `WebServer.exe` from `dist/ProjectBundle`; compiled server serves from `web/static` and `_internal/web/static`.
- Launcher restarts itself instead of starting GUI/server:
  - Use `Launcher.exe` from `dist/ProjectBundle` (same folder as `ImageDetectionBot.exe` and `WebServer.exe`).
- Web-start doesn’t show current step in GUI:
  - GUI switches to Sequences, selects the active sequence, and brings window to foreground.
- “MOVE action requires x and y coordinates” after web-start:
  - MOVE without `x,y` uses the detected position in the step (treated like MOVE_TO) to keep actions running.
- Random movement not saved in web failsafe:
  - Web editors save `random` and `random_region`; compiled GUI honors random move_to and random-region clicks.
- MJPEG preview shows `net::ERR_ABORTED` occasionally:
  - That’s a reconnect artifact; it does not affect saving, IPC, or UI updates.
