import os
import sys
import json
import logging
import tempfile
import cv2
import numpy as np
import pyautogui
import shutil
from datetime import datetime
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QListWidget, QStackedWidget, QLineEdit, QSpinBox, 
                            QDoubleSpinBox, QComboBox, QFileDialog, QMessageBox, QCheckBox, 
                            QGroupBox, QScrollArea, QSplitter, QFrame, QSizePolicy, QToolBar, 
                            QStatusBar, QProgressBar, QDialog, QFormLayout, QListWidgetItem, QInputDialog, QMenu,
                            QDialogButtonBox, QMenuBar, QTableWidget, QTableWidgetItem, QGridLayout,
                            QHeaderView, QAbstractItemView, QTabWidget, QTimeEdit)
from PyQt6.QtCore import Qt, QSize, QTimer, pyqtSignal, QThread, QObject, QPoint, QRect, QEvent
from PyQt6.QtGui import (QAction, QIcon, QPixmap, QImage, QPainter, QPen, QColor, 
                        QScreen, QGuiApplication, QKeySequence, QShortcut, QKeyEvent)
import pyautogui
import numpy as np
from PIL import ImageGrab, Image
import darkdetect

# Import the bot functionality
from image_detection_bot import ImageDetectionBot, ActionType, Action, parse_action

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class WorkerSignals(QObject):
    """Defines the signals available from a running worker thread."""
    finished = pyqtSignal()
    error = pyqtSignal(str)
    result = pyqtSignal(object)
    update = pyqtSignal(dict)

import os
import signal

class BotWorker(QThread):
    """Worker thread for running bot operations."""
    def __init__(self, bot, sequence, loop=False, loop_count=1, non_required_wait=False, failsafe_config=None, failsafe_only=False, max_runtime_seconds=None, groups=None):
        super().__init__()
        self.bot = bot
        self.sequence = sequence
        self.loop = loop
        self.loop_count = loop_count if loop else 1
        self.non_required_wait = non_required_wait  # Whether to wait for non-required steps
        self.failsafe_config = failsafe_config  # New failsafe configuration
        self.failsafe_only = failsafe_only  # If True, run only the failsafe sequence and exit
        self.max_runtime_seconds = max_runtime_seconds  # Max runtime cutoff
        self.groups = groups or {}
        self.signals = WorkerSignals()
        self._is_running = True
        self._should_stop = False  # Flag to indicate if we should stop
        self.current_step = 0
        self.total_steps = 0
        self.current_iteration = 0
        self._process_pid = None  # Store the process ID for forceful termination
        self._executing_failsafe = False  # Flag to track if we're in failsafe execution
        logger.info("BotWorker initialized with loop=%s, loop_count=%s, non_required_wait=%s, failsafe_config=%s", 
                    loop, loop_count, non_required_wait, bool(failsafe_config))
    
    def run(self):
        """Run the sequence of steps with optional looping."""
        # Store the process ID for forceful termination if needed
        self._process_pid = os.getpid()
        logger.info(f"BotWorker started with PID: {self._process_pid}")
        # Track start time for max runtime cutoff
        import time
        self._start_time = time.time()
        
        # Failsafe configuration is now handled by self.failsafe_config
        # If we're in failsafe-only mode (e.g., Test Failsafe button), execute and exit
        if getattr(self, 'failsafe_only', False):
            try:
                self.signals.update.emit({
                    "status": "Starting failsafe test...",
                    "progress": 0
                })
                self.execute_failsafe_sequence()
                self.signals.update.emit({
                    "status": "Failsafe sequence completed.",
                    "progress": 100
                })
            except Exception as e:
                self.signals.error.emit(f"Error executing failsafe sequence: {str(e)}")
            finally:
                self._is_running = False
                self.signals.finished.emit()
                return
        
        try:
            while self._is_running and not self._should_stop and (not self.loop or self.current_iteration < self.loop_count):
                # Check max runtime before starting iteration
                if self.max_runtime_seconds is not None:
                    elapsed = time.time() - getattr(self, '_start_time', time.time())
                    if elapsed >= self.max_runtime_seconds:
                        logger.info("Max runtime reached (%.2fs >= %.2fs)", elapsed, self.max_runtime_seconds)
                        self.signals.update.emit({
                            "status": "Max runtime reached. Stopping...",
                            "progress": int(100)
                        })
                        self._should_stop = True
                        break
                self.current_iteration += 1
                logger.info("Starting iteration %d/%d", self.current_iteration, self.loop_count if self.loop else 1)
                
                # Check for stop request before starting iteration
                if self._should_stop:
                    logger.info("Stop requested before starting iteration")
                    break
                # Expand groups inline for this iteration
                base_steps = self.sequence.get('steps', [])
                expanded_steps = []
                for st in base_steps:
                    try:
                        if isinstance(st, dict) and 'call_group' in st:
                            grp_name = st.get('call_group')
                            grp_steps = []
                            if grp_name and grp_name in self.groups:
                                grp_steps = self.groups.get(grp_name) or []
                            else:
                                logger.warning(f"Group '{grp_name}' not found; skipping call.")
                            for gs in grp_steps:
                                expanded_steps.append(gs)
                        else:
                            expanded_steps.append(st)
                    except Exception as e:
                        logger.error(f"Error expanding group step: {e}")
                        expanded_steps.append(st)

                self.total_steps = len(expanded_steps)
                if self.total_steps == 0:
                    self.signals.error.emit("No steps in sequence")
                    return
                
                iteration_text = f" ({self.current_iteration}/{self.loop_count})" if self.loop and self.loop_count > 1 else ""
                self.signals.update.emit({
                    "status": f"Starting sequence '{self.sequence.get('name', 'Unnamed')}'" + 
                             (f" (Iteration: {self.current_iteration}{iteration_text})" if self.loop or self.current_iteration > 1 else "") + 
                             f" with {self.total_steps} steps...",
                    "progress": 0
                })
                
                # Track the last position set by a MOVE action
                last_move_position = None
                should_continue = True
                
                try:
                    for i, step in enumerate(expanded_steps):
                        if not self._is_running:
                            should_continue = False
                            break
                        # Check max runtime inside steps loop
                        if self.max_runtime_seconds is not None:
                            elapsed = time.time() - getattr(self, '_start_time', time.time())
                            if elapsed >= self.max_runtime_seconds:
                                logger.info("Max runtime reached during steps (%.2fs >= %.2fs)", elapsed, self.max_runtime_seconds)
                                self.signals.update.emit({
                                    "status": "Max runtime reached during steps. Stopping...",
                                    "progress": int(100)
                                })
                                self._should_stop = True
                                should_continue = False
                                break
                        
                        self.current_step = i
                        template_name = step.get('find', '')
                        required = step.get('required', True)
                        timeout = step.get('timeout', 10.0)
                        confidence = step.get('confidence', 0.9)
                        search_region = step.get('search_region')
                        # If no explicit region, use per-step monitor override if provided
                        mon = step.get('monitor')
                        if not search_region and mon and ((isinstance(mon, tuple) and len(mon) == 4) or (isinstance(mon, list) and len(mon) == 4)):
                            search_region = tuple(mon)
                        step_loop_count = step.get('loop_count', 1)
                        
                        # Check for stop request before each step
                        if self._should_stop:
                            logger.info("Stop requested, breaking step loop")
                            should_continue = False
                            break
                        
                        # Update status
                        self.signals.update.emit({
                            "status": f"Step {i+1}/{self.total_steps}: Looking for '{template_name}'...",
                            "current_step": template_name,
                            "progress": int((i / self.total_steps) * 100)
                        })
                        
                        for step_loop in range(step_loop_count):
                            if self._should_stop:
                                logger.info("Stop requested, breaking step loop")
                                should_continue = False
                                break
                            
                            # Handle actions without template
                            if not template_name or str(template_name).lower() == 'none':
                                actions = step.get('actions', [])
                                for action_data in actions:
                                    try:
                                        action = parse_action(action_data)
                                        success = self.bot.execute_action(action)
                                        if action.type == ActionType.MOVE and action.x is not None and action.y is not None:
                                            last_move_position = (action.x, action.y)
                                        if not success:
                                            self.signals.error.emit(f"Failed to execute {action.type.value}")
                                            should_continue = False
                                            break
                                        # --- FAILSAFE CHECK ---
                                        if self.check_failsafe_trigger():
                                            # Failsafe sequence was executed, continue with next step
                                            break
                                    except Exception as e:
                                        self.signals.error.emit(f"Error executing action: {str(e)}")
                                        should_continue = False
                                        break
                                if not should_continue:
                                    break
                                # If failsafe triggered, continue at new step
                                if failsafe_template is not None and failsafe_goto_step is not None and i == failsafe_goto_step:
                                    continue
                            else:
                                # Find the template in the image for each loop iteration
                                position = None
                                try:
                                    if self._should_stop:
                                        logger.info("Stop requested, breaking before find operation")
                                        should_continue = False
                                        break
                                    if search_region:
                                        # For non-required steps, use 0.5s timeout if waiting is disabled
                                        current_timeout = timeout if (required or self.non_required_wait) else 0.5
                                        logger.info(f"Looking for template '{template_name}' with timeout={current_timeout}s (required={required}, non_required_wait={self.non_required_wait})")
                                        strategy = step.get('strategy')  # e.g., 'feature' or None
                                        min_inliers = step.get('min_inliers', 12)
                                        ratio_thresh = step.get('ratio_thresh', 0.75)
                                        ransac_thresh = step.get('ransac_thresh', 4.0)
                                        position = self.bot.find_image(
                                            template_name=template_name,
                                            region=search_region,
                                            timeout=current_timeout,
                                            confidence=confidence,
                                            strategy=strategy,
                                            min_inliers=min_inliers,
                                            ratio_thresh=ratio_thresh,
                                            ransac_thresh=ransac_thresh
                                        )
                                    else:
                                        start_time = datetime.now()
                                        check_interval = 0.1
                                        # For non-required steps, use 0.5s timeout if waiting is disabled
                                        current_timeout = timeout if (required or self.non_required_wait) else 0.5
                                        logger.info(f"Looking for template '{template_name}' with timeout={current_timeout}s (required={required}, non_required_wait={self.non_required_wait})")
                                        strategy = step.get('strategy')
                                        min_inliers = step.get('min_inliers', 12)
                                        ratio_thresh = step.get('ratio_thresh', 0.75)
                                        ransac_thresh = step.get('ransac_thresh', 4.0)
                                        while not self._should_stop and (datetime.now() - start_time).total_seconds() < current_timeout:
                                            position = self.bot.find_image(
                                                template_name=template_name,
                                                confidence=confidence,
                                                strategy=strategy,
                                                min_inliers=min_inliers,
                                                ratio_thresh=ratio_thresh,
                                                ransac_thresh=ransac_thresh
                                            )
                                            if position is not None:
                                                break
                                            self.msleep(int(check_interval * 1000))
                                        if self._should_stop:
                                            logger.info("Stop requested during wait_for_image")
                                            should_continue = False
                                            break
                                    if position is None:
                                        if required:
                                            self.signals.error.emit(f"Required template '{template_name}' not found")
                                            should_continue = False
                                            break
                                        else:
                                            self.signals.update.emit({
                                                "status": f"Optional template '{template_name}' not found, skipping...",
                                                "progress": int(((i + 0.5) / self.total_steps) * 100)
                                            })
                                            break
                                    # Execute actions for this step (only once per template detection)
                                    actions = step.get('actions', [])
                                    for action_data in actions:
                                        if not self._is_running:
                                            should_continue = False
                                            break
                                        try:
                                            action = parse_action(action_data)
                                            self.signals.update.emit({
                                                "status": f"Executing {action.type.value}...",
                                                "progress": int(((i + 0.5) / self.total_steps) * 100)
                                            })
                                            if action.type == ActionType.MOVE and action.x is not None and action.y is not None:
                                                last_move_position = (action.x, action.y)
                                            if action.type == ActionType.MOVE:
                                                if action.x is not None and action.y is not None:
                                                    success = self.bot.execute_action(action)
                                                elif position is not None:
                                                    # Treat MOVE as MOVE_TO when a detected position is available
                                                    success = self.bot.execute_action_at_position(action, position)
                                                else:
                                                    self.signals.error.emit("MOVE action requires x and y coordinates")
                                                    should_continue = False
                                                    break
                                            elif action.type == ActionType.MOVE_TO and position is not None:
                                                success = self.bot.execute_action_at_position(action, position)
                                            elif action.type == ActionType.CLICK and last_move_position is not None:
                                                success = self.bot.execute_action_at_position(action, last_move_position)
                                            elif position is not None:
                                                success = self.bot.execute_action_at_position(action, position)
                                            else:
                                                success = self.bot.execute_action(action)
                                            # Log the result of each action
                                            logger.info(f"Action {action.type.value} executed with result: {success}")
                                            if not success:
                                                if action.type == ActionType.WAIT:
                                                    logger.warning(f"Wait action failed, but continuing to next action/loop.")
                                                    continue
                                                self.signals.error.emit(f"Failed to execute {action.type.value}")
                                                should_continue = False
                                                break
                                            # --- FAILSAFE CHECK ---
                                            if self.check_failsafe_trigger():
                                                # Failsafe sequence was executed, continue with next step
                                                break
                                        except Exception as e:
                                            logger.error(f"Exception during action {action_data.get('type')}: {str(e)}")
                                            if action_data.get('type') == 'wait':
                                                logger.warning(f"Wait action exception, but continuing to next action/loop.")
                                                continue
                                            self.signals.error.emit(f"Error executing action: {str(e)}")
                                            should_continue = False
                                            break
                                    if not should_continue:
                                        break
                                except Exception as e:
                                    self.signals.error.emit(f"Error finding template '{template_name}': {str(e)}")
                                    should_continue = False
                                    break
                        if not should_continue:
                            break
                        
                    # End of steps loop
                    
                    if not should_continue or not self.loop:
                        break
                        
                    # Small delay between iterations
                    import time
                    time.sleep(0.1)
                    
                except Exception as e:
                    self.signals.error.emit(f"Error in sequence execution: {str(e)}")
                    should_continue = False
                
                # Update completion status
                if self._is_running and should_continue and self.loop:
                    self.signals.update.emit({
                        "status": f"Completed iteration {self.current_iteration}, starting next...",
                        "progress": 0
                    })
                
            # End of main loop
            
            if self._is_running:
                self.signals.update.emit({
                    "status": f"Sequence '{self.sequence.get('name', 'Unnamed')}' completed " +
                             (f"after {self.current_iteration} iterations" if self.loop and self.current_iteration > 1 else "successfully") + "!",
                    "progress": 100
                })
            
            self.signals.finished.emit()
            
        except Exception as e:
            import traceback
            error_msg = f"Error in sequence execution: {str(e)}\n\n{traceback.format_exc()}"
            self.signals.error.emit(error_msg)
        finally:
            self._is_running = False
            self.signals.finished.emit()
            self.wait()

    def execute_failsafe_sequence(self):
        """Execute the failsafe sequence when the failsafe template is detected."""
        if not isinstance(self.failsafe_config, dict) or not self.failsafe_config.get('enabled'):
            return False
            
        failsafe_sequence = self.failsafe_config.get('sequence', [])
        # Normalize sequence format if provided as {"steps": [...]} from editor
        if isinstance(failsafe_sequence, dict):
            failsafe_sequence = failsafe_sequence.get('steps', [])
        if not isinstance(failsafe_sequence, list):
            logger.warning("Invalid failsafe sequence format; expected a list of steps")
            return False
        if not failsafe_sequence:
            return False
            
        logger.info("Executing failsafe sequence")
        self._executing_failsafe = True
        
        try:
            self.signals.update.emit({
                "status": "Failsafe triggered! Executing failsafe sequence...",
                "progress": 0
            })
            
            # Expand group call steps inline
            expanded = []
            try:
                grp_map = getattr(self, 'groups', {}) or {}
                for st in failsafe_sequence:
                    if isinstance(st, dict) and 'call_group' in st:
                        gname = st.get('call_group')
                        gsteps = []
                        if gname and gname in grp_map:
                            grp = grp_map.get(gname)
                            gsteps = grp if isinstance(grp, list) else (grp.get('steps', []) if isinstance(grp, dict) else [])
                            try:
                                logger.info(f"Failsafe: expanding group '{gname}' into {len(gsteps)} steps")
                            except Exception:
                                pass
                        else:
                            logger.warning(f"Group '{gname}' not found for failsafe call; skipping.")
                        expanded.extend(gsteps)
                    else:
                        expanded.append(st)
            except Exception as e:
                logger.debug(f"Failsafe group expansion error: {e}")
                expanded = failsafe_sequence
            try:
                logger.info(f"Failsafe: expanded sequence size {len(expanded)} (original {len(failsafe_sequence)})")
            except Exception:
                pass

            for i, step in enumerate(expanded):
                if not self._is_running or self._should_stop:
                    break
                    
                self.signals.update.emit({
                    "status": f"Failsafe step {i+1}/{len(failsafe_sequence)}: {step.get('name', 'Unnamed step')}",
                    "progress": int((i / len(failsafe_sequence)) * 100)
                })
                
                # Execute the failsafe step
                template_name = step.get('find', '')
                actions = step.get('actions', [])

                if template_name and str(template_name).lower() != 'none':
                    # Find template first
                    # Accept both desktop and web editor keys
                    strategy = step.get('strategy') or step.get('detection_strategy')
                    region = step.get('search_region') or step.get('region')
                    mon = step.get('monitor')
                    if not region and mon and ((isinstance(mon, tuple) and len(mon) == 4) or (isinstance(mon, list) and len(mon) == 4)):
                        region = tuple(mon)
                    position = self.bot.find_image(
                        template_name=template_name,
                        region=region,
                        timeout=step.get('timeout', 5.0),
                        confidence=step.get('confidence', 0.9),
                        strategy=strategy,
                        min_inliers=step.get('min_inliers', 12),
                        ratio_thresh=step.get('ratio_thresh', step.get('ratio', 0.75)),
                        ransac_thresh=step.get('ransac_thresh', step.get('ransac', 4.0))
                    )

                    if position is None and step.get('required', True):
                        logger.warning(f"Required template '{template_name}' not found in failsafe sequence")
                        continue

                    # Execute actions at found position
                    for action_data in actions:
                        if not self._is_running or self._should_stop:
                            break
                        try:
                            action = parse_action(action_data)
                            if position is not None and action.type in [ActionType.CLICK, ActionType.MOVE_TO]:
                                success = self.bot.execute_action_at_position(action, position)
                            else:
                                success = self.bot.execute_action(action)
                            if not success:
                                logger.warning(f"Failed to execute {action.type.value} in failsafe sequence")
                        except Exception as e:
                            logger.error(f"Error executing failsafe action: {str(e)}")

                else:
                    # Execute actions without template
                    for action_data in actions:
                        if not self._is_running or self._should_stop:
                            break
                            
                        try:
                            action = parse_action(action_data)
                            success = self.bot.execute_action(action)
                            if not success:
                                logger.warning(f"Failed to execute {action.type.value} in failsafe sequence")
                                
                        except Exception as e:
                            logger.error(f"Error executing failsafe action: {str(e)}")
            
            self.signals.update.emit({
                "status": "Failsafe sequence completed",
                "progress": 100
            })
            
            return True
            
        except Exception as e:
            logger.error(f"Error executing failsafe sequence: {str(e)}")
            self.signals.error.emit(f"Error in failsafe sequence: {str(e)}")
            return False
            
        finally:
            self._executing_failsafe = False

    def check_failsafe_trigger(self):
        """Check if the failsafe template is detected and trigger failsafe sequence if found."""
        if not self.failsafe_config or not self.failsafe_config.get('enabled'):
            return False
            
        if self._executing_failsafe:  # Don't trigger failsafe while already executing it
            return False
            
        failsafe_template = self.failsafe_config.get('template')
        failsafe_confidence = self.failsafe_config.get('confidence', 0.8)
        failsafe_region = self.failsafe_config.get('region')
        
        if not failsafe_template:
            return False
            
        # Quick check for failsafe template
        pos = self.bot.find_image(
            template_name=failsafe_template,
            region=failsafe_region,
            timeout=0.1,  # Very short timeout for quick checks
            confidence=failsafe_confidence
        )
        
        if pos is not None:
            logger.info(f"Failsafe template '{failsafe_template}' detected at {pos}")
            return self.execute_failsafe_sequence()
            
        return False

    def stop(self):
        """Request the worker to stop."""
        logger.info("Stop requested for BotWorker")
        self._is_running = False
        self._should_stop = True
        
        # Try graceful stop first
        if hasattr(self.bot, 'stop'):
            self.bot.stop()  # If the bot has a stop method, call it
        
        # Force stop after a short delay if needed
        if not self.wait(1000):  # Wait up to 1 second for graceful stop
            logger.warning("Graceful stop failed, forcing termination")
            self.terminate()
            if not self.wait(500):  # Wait up to 0.5 seconds for terminate to complete
                logger.error("Failed to terminate thread, trying process kill")
                if self._process_pid:
                    try:
                        os.kill(self._process_pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass  # Process already dead
                    except Exception as e:
                        logger.error(f"Error killing process: {e}")
        
        logger.info("Worker stop completed")

class ScreenCaptureDialog(QDialog):
    """Dialog for capturing a region of the screen.
    Optionally restricts to a specific monitor geometry if provided.
    """
    def __init__(self, parent=None, target_geometry: QRect | None = None):
        super().__init__(parent)
        self.setWindowTitle("Capture Screen Region")
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint | 
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool
        )
        self.setCursor(Qt.CursorShape.CrossCursor)
        
        # Make window semi-transparent
        self.setWindowOpacity(0.3)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        
        # Store screen information
        self.screens = QGuiApplication.screens()
        self.current_screen = QGuiApplication.primaryScreen()
        self.screen_geometry = self.current_screen.availableGeometry()
        self.target_geometry = target_geometry
        
        # Calculate combined geometry of all screens
        self.combined_geometry = QRect()
        for screen in self.screens:
            self.combined_geometry = self.combined_geometry.united(screen.geometry())
        
        # Set dialog to cover target monitor or all screens
        if isinstance(self.target_geometry, QRect) and self.target_geometry.isValid():
            self.setGeometry(self.target_geometry)
            self.move(self.target_geometry.topLeft())
        else:
            self.setGeometry(self.combined_geometry)
            self.move(self.combined_geometry.topLeft())
        
        # Selection rectangle in screen coordinates
        self.start_pos = None
        self.end_pos = None
        self.selection_rect = None
        
    def paintEvent(self, event):
        """Draw the semi-transparent overlay and selection rectangle."""
        try:
            painter = QPainter(self)
            
            # Draw semi-transparent overlay for all screens
            painter.setBrush(QColor(0, 0, 0, 100))  # Semi-transparent black
            painter.setPen(Qt.PenStyle.NoPen)
            
            # Fill overlay over the entire dialog window region
            painter.drawRect(self.rect())
            
            # Draw the selection rectangle if we have valid points
            if (self.start_pos and self.end_pos and 
                not self.start_pos.isNull() and not self.end_pos.isNull() and
                self.selection_rect and self.selection_rect.isValid()):
                
                # Draw the selection border
                painter.setPen(QPen(Qt.GlobalColor.red, 2, Qt.PenStyle.SolidLine))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                
                # Convert to local coordinates relative to this dialog window
                local_origin = self.geometry().topLeft()
                local_rect = QRect(self.selection_rect.topLeft() - local_origin, self.selection_rect.size())
                painter.drawRect(local_rect)
                
                # Draw size info
                size_text = f"{self.selection_rect.width()} x {self.selection_rect.height()}"
                text_rect = painter.fontMetrics().boundingRect(size_text)
                text_rect.moveBottomLeft(local_rect.bottomLeft() + QPoint(0, 5))
                text_rect.adjust(-5, -2, 5, 2)  # Add some padding
                
                # Ensure text is visible
                painter.fillRect(text_rect, QColor(0, 0, 0, 180))
                painter.setPen(Qt.GlobalColor.white)
                painter.drawText(text_rect, Qt.AlignmentFlag.AlignCenter, size_text)
            
            painter.end()
        except Exception as e:
            print(f"Error in paintEvent: {e}")
            if painter.isActive():
                painter.end()
    
    def mousePressEvent(self, event):
        """Start selection on mouse press."""
        if event.button() == Qt.MouseButton.LeftButton:
            self.start_pos = event.globalPosition().toPoint()
            self.end_pos = self.start_pos
            self.selection_rect = QRect(self.start_pos, QSize(1, 1))
            self.update()
    
    def mouseMoveEvent(self, event):
        """Update selection rectangle on mouse move."""
        if self.start_pos:
            self.end_pos = event.globalPosition().toPoint()
            
            # Ensure we have valid coordinates
            if not self.end_pos.isNull():
                # Create a normalized rectangle from start to current position
                x1 = min(self.start_pos.x(), self.end_pos.x())
                y1 = min(self.start_pos.y(), self.end_pos.y())
                x2 = max(self.start_pos.x(), self.end_pos.x())
                y2 = max(self.start_pos.y(), self.end_pos.y())
                
                # Ensure minimum size
                if x2 - x1 < 5: x2 = x1 + 5
                if y2 - y1 < 5: y2 = y1 + 5
                
                self.selection_rect = QRect(
                    QPoint(x1, y1),
                    QPoint(x2, y2)
                )
                self.update()
    
    def mouseReleaseEvent(self, event):
        """Finish selection on mouse release."""
        if event.button() == Qt.MouseButton.LeftButton and self.start_pos:
            self.end_pos = event.globalPosition().toPoint()
            
            # Only accept if the selection is big enough
            if (abs(self.end_pos.x() - self.start_pos.x()) > 5 and 
                abs(self.end_pos.y() - self.start_pos.y()) > 5):
                self.accept()
            else:
                self.reject()
    
    def keyPressEvent(self, event):
        """Handle Escape key to cancel."""
        if event.key() == Qt.Key.Key_Escape:
            self.reject()
    
    def get_capture_rect(self) -> QRect:
        """Get the captured rectangle in screen coordinates."""
        if self.selection_rect and self.selection_rect.isValid():
            # Ensure the rectangle is within all screens' combined bounds
            bounds = self.target_geometry if (isinstance(self.target_geometry, QRect) and self.target_geometry.isValid()) else self.combined_geometry
            rect = self.selection_rect.intersected(bounds)
            
            # Convert to screen coordinates if needed
            if rect.isValid() and rect.width() > 5 and rect.height() > 5:  # Minimum size
                return rect
        return None

class OverlayPreviewWindow(QDialog):
    """Non-interactive transparent overlay to preview a region or a click point.
    Covers either a target monitor geometry or the combined desktop, draws a red
    rectangle or crosshair marker, and auto-closes after a short duration.
    """
    def __init__(self, parent=None, rect: QRect | None = None, point: QPoint | None = None, target_geometry: QRect | None = None, duration_ms: int = 1200, message: str | None = None):
        super().__init__(parent)
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool
        )
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setWindowOpacity(0.3)
        self._rect = rect if (isinstance(rect, QRect) and rect.isValid()) else None
        self._point = point
        self._message = message

        # Determine coverage geometry
        screens = QGuiApplication.screens()
        combined = QRect()
        for sc in screens:
            combined = combined.united(sc.geometry())
        if isinstance(target_geometry, QRect) and target_geometry.isValid():
            self._coverage = target_geometry
        else:
            self._coverage = combined
        self.setGeometry(self._coverage)
        self.move(self._coverage.topLeft())

        # Auto-close shortly
        try:
            self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        except Exception:
            pass
        QTimer.singleShot(max(300, int(duration_ms)), self.close)

    def paintEvent(self, event):
        try:
            painter = QPainter(self)
            # Dim background
            painter.setBrush(QColor(0, 0, 0, 100))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRect(self.rect())

            origin = self.geometry().topLeft()

            # Draw rectangle preview
            if self._rect and self._rect.isValid():
                local_rect = QRect(self._rect.topLeft() - origin, self._rect.size())
                painter.setPen(QPen(Qt.GlobalColor.red, 2, Qt.PenStyle.SolidLine))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawRect(local_rect)
                # Size text box
                size_text = f"{self._rect.width()} x {self._rect.height()}"
                text_rect = painter.fontMetrics().boundingRect(size_text)
                text_rect.moveBottomLeft(local_rect.bottomLeft() + QPoint(0, 5))
                text_rect.adjust(-5, -2, 5, 2)
                painter.fillRect(text_rect, QColor(0, 0, 0, 180))
                painter.setPen(Qt.GlobalColor.white)
                painter.drawText(text_rect, Qt.AlignmentFlag.AlignCenter, size_text)

            # Draw click point preview
            if isinstance(self._point, QPoint):
                local_pt = self._point - origin
                painter.setPen(QPen(Qt.GlobalColor.green, 2, Qt.PenStyle.SolidLine))
                # Crosshair
                painter.drawLine(local_pt + QPoint(-10, 0), local_pt + QPoint(10, 0))
                painter.drawLine(local_pt + QPoint(0, -10), local_pt + QPoint(0, 10))
                painter.setBrush(QColor(0, 255, 0, 120))
                painter.drawEllipse(local_pt, 6, 6)

            # Optional message centered
            if isinstance(self._message, str) and self._message:
                painter.setPen(Qt.GlobalColor.white)
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self._message)
            painter.end()
        except Exception:
            try:
                if painter.isActive():
                    painter.end()
            except Exception:
                pass

class MonitorInfoDialog(QDialog):
    """Dialog to display monitor geometries and DPI, with quick capture preview per monitor."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Monitor Info")
        self.setMinimumSize(500, 300)
        layout = QVBoxLayout(self)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Index", "X", "Y", "Size", "DPI Ratio"])
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table)
        btns = QHBoxLayout()
        self.capture_btn = QPushButton("Capture All")
        self.capture_btn.clicked.connect(self.capture_all)
        btns.addWidget(self.capture_btn)
        btns.addStretch()
        layout.addLayout(btns)
        self.preview_label = QLabel()
        self.preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_label.setMinimumHeight(150)
        layout.addWidget(self.preview_label)
        self.populate()

    def populate(self):
        screens = QGuiApplication.screens()
        self.table.setRowCount(len(screens))
        for idx, sc in enumerate(screens):
            g = sc.geometry()
            dpr = sc.devicePixelRatio()
            self.table.setItem(idx, 0, QTableWidgetItem(str(idx + 1)))
            self.table.setItem(idx, 1, QTableWidgetItem(str(g.x())))
            self.table.setItem(idx, 2, QTableWidgetItem(str(g.y())))
            self.table.setItem(idx, 3, QTableWidgetItem(f"{g.width()}x{g.height()}"))
            self.table.setItem(idx, 4, QTableWidgetItem(f"{dpr:.2f}"))

    def capture_all(self):
        try:
            screens = QGuiApplication.screens()
            if not screens:
                return
            x0 = min(sc.geometry().x() for sc in screens)
            y0 = min(sc.geometry().y() for sc in screens)
            x1 = max(sc.geometry().x() + sc.geometry().width() for sc in screens)
            y1 = max(sc.geometry().y() + sc.geometry().height() for sc in screens)
            total_w, total_h = x1 - x0, y1 - y0
            canvas = np.zeros((total_h, total_w, 3), dtype=np.uint8)
            for sc in screens:
                g = sc.geometry()
                pm = sc.grabWindow(0)
                if pm.isNull():
                    continue
                qi = pm.toImage().convertToFormat(QImage.Format.Format_BGR888)
                h = qi.height(); w = qi.width(); bpl = qi.bytesPerLine()
                size_bytes = getattr(qi, 'sizeInBytes', None)
                size_bytes = size_bytes() if callable(size_bytes) else (bpl * h)
                bits = qi.bits(); bits.setsize(size_bytes)
                buf = np.frombuffer(bits, dtype=np.uint8)
                img_row = buf.reshape((h, bpl))
                img_row = img_row[:, :w * 3]
                arr = img_row.reshape((h, w, 3))
                off_x = g.x() - x0
                off_y = g.y() - y0
                y_end = min(off_y + h, total_h)
                x_end = min(off_x + w, total_w)
                canvas[off_y:y_end, off_x:x_end] = arr[:y_end - off_y, :x_end - off_x]
            # Show preview
            height, width, channel = canvas.shape
            q_img = QImage(canvas.data, width, height, width * 3, QImage.Format.Format_BGR888)
            self.preview_label.setPixmap(QPixmap.fromImage(q_img).scaled(
                self.preview_label.width(), self.preview_label.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            ))
        except Exception as e:
            QMessageBox.warning(self, "Capture", f"Failed to capture monitors: {e}")


class TemplatePreview(QWidget):
    """Widget to display and manage a single template."""
    def __init__(self, name: str, path: str, parent=None):
        super().__init__(parent)
        self.name = name
        self.path = path
        self.images_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")
        os.makedirs(self.images_dir, exist_ok=True)
        self.name_edit = None
        self.path_edit = None
        self.preview_label = None
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)  # Set layout on self
        
        # Template name and path
        self.name_edit = QLineEdit()
        self.name_edit.setText(str(self.name))  # Ensure name is a string
        self.path_edit = QLineEdit()
        self.path_edit.setText(str(self.path))  # Ensure path is a string
        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self.browse_image)
        
        # Preview image
        self.preview_label = QLabel()
        self.preview_label.setFixedSize(100, 80)
        self.preview_label.setStyleSheet("border: 1px solid #ccc;")
        self.update_preview()
        
        # Layout
        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("Name:"))
        path_layout.addWidget(self.name_edit)
        
        file_layout = QHBoxLayout()
        file_layout.addWidget(QLabel("Path:"))
        file_layout.addWidget(self.path_edit)
        file_layout.addWidget(browse_btn)
        
        layout.addLayout(path_layout)
        layout.addLayout(file_layout)
        layout.addWidget(QLabel("Preview:"))
        layout.addWidget(self.preview_label, alignment=Qt.AlignmentFlag.AlignCenter)
        
        self.setLayout(layout)
    
    def browse_image(self):
        """Open file dialog to select an image or capture from screen."""
        menu = QMenu(self)
        
        # Add options to capture from screen or select file
        capture_action = QAction("Capture from Screen", self)
        capture_action.triggered.connect(self.capture_from_screen)
        
        select_action = QAction("Select from File", self)
        select_action.triggered.connect(self.select_from_file)
        
        menu.addAction(capture_action)
        menu.addAction(select_action)
        
        # Show the menu at the button position
        menu.exec_(self.sender().mapToGlobal(QPoint(0, self.sender().height())))
    
    def capture_from_screen(self):
        """Capture a region of the screen."""
        try:
            # First, capture the screen region
            # Restrict overlay to selected monitor if applicable
            target_geometry = None
            try:
                # MainWindow is the parent of the QDialog that owns this preview
                main_window = self.parent().parent() if self.parent() else None
                if hasattr(main_window, 'monitor_combo'):
                    data = main_window.monitor_combo.currentData()
                    if isinstance(data, tuple) and len(data) == 4:
                        x, y, w, h = data
                        target_geometry = QRect(x, y, w, h)
                    # 'ALL' or None → full desktop
            except Exception:
                pass
            capture_dialog = ScreenCaptureDialog(self, target_geometry=target_geometry)
            if capture_dialog.exec() != QDialog.DialogCode.Accepted:
                return False
                
            rect = capture_dialog.get_capture_rect()
            if not rect or not rect.isValid():
                return False
                
            try:
                # Capture the screen region using QScreen.grabWindow with robust monitor selection
                screens = QGuiApplication.screens()
                # Choose screen with largest intersection area with rect
                best = None
                best_area = -1
                for sc in screens:
                    g = sc.geometry()
                    gx1, gy1, gx2, gy2 = g.x(), g.y(), g.x() + g.width(), g.y() + g.height()
                    ix1, iy1 = max(rect.x(), gx1), max(rect.y(), gy1)
                    ix2, iy2 = min(rect.x() + rect.width(), gx2), min(rect.y() + rect.height(), gy2)
                    if ix2 > ix1 and iy2 > iy1:
                        area = (ix2 - ix1) * (iy2 - iy1)
                        if area > best_area:
                            best_area = area
                            best = sc
                target = best if best is not None else QGuiApplication.primaryScreen()
                geom = target.geometry()
                local_x = max(0, rect.x() - geom.x())
                local_y = max(0, rect.y() - geom.y())
                grab_w = max(1, min(rect.width(), geom.width() - local_x))
                grab_h = max(1, min(rect.height(), geom.height() - local_y))
                dpr = target.devicePixelRatio()
                pm = target.grabWindow(0, int(local_x * dpr), int(local_y * dpr), int(grab_w * dpr), int(grab_h * dpr))
                
                # Create a temporary file for the screenshot
                temp_dir = os.path.join(self.images_dir, "temp")
                os.makedirs(temp_dir, exist_ok=True)
                temp_filename = f"temp_{int(datetime.now().timestamp())}.png"
                temp_path = os.path.join(temp_dir, temp_filename)
                
                # Save the screenshot pixmap directly to PNG (fallback to PIL/pyautogui if null)
                if pm and not pm.isNull():
                    pm.save(temp_path, "PNG")
                else:
                    try:
                        screenshot = ImageGrab.grab(bbox=(rect.x(), rect.y(), rect.x() + rect.width(), rect.y() + rect.height()))
                    except Exception:
                        screenshot = pyautogui.screenshot(region=(rect.x(), rect.y(), rect.width(), rect.height()))
                    screenshot.save(temp_path, "PNG")
                
                # Update the UI
                if os.path.exists(temp_path):
                    self.path = temp_path
                    self.path_edit.setText(temp_path)
                    
                    # Generate a default name if none exists
                    if not self.name_edit.text() or self.name_edit.text().startswith("template_"):
                        self.name_edit.setText(f"template_{int(datetime.now().timestamp())}")
                    
                    self.update_preview()
                    self.name_edit.setFocus()
                    return True
                
                return False
                
            except Exception as capture_error:
                QMessageBox.critical(self, "Capture Error", f"Failed to process screenshot: {str(capture_error)}")
                return False
                
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to capture screen: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def select_from_file(self):
        """Select an image file from disk."""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select Template Image", "", 
            "Images (*.png *.jpg *.jpeg *.bmp)")
            
        if file_path:
            # Normalize paths for comparison
            file_path = os.path.normpath(file_path)
            images_dir = os.path.normpath(self.images_dir)
            
            # If the file is not in the images directory or has a different case, copy it there
            if not os.path.normcase(file_path).startswith(os.path.normcase(images_dir) + os.sep):
                # Get a base name for the file
                base_name = os.path.basename(file_path)
                name, ext = os.path.splitext(base_name)
                
                # Ask for template name
                new_name, ok = QInputDialog.getText(
                    self,  # Add parent
                    "Template Name", 
                    "Enter a name for this template:",
                    text=name
                )
                
                if ok and new_name:
                    # Copy the file to the images directory
                    dest_path = os.path.join(self.images_dir, f"{new_name}{ext}")
                    try:
                        shutil.copy2(file_path, dest_path)
                        # Update UI
                        self.name_edit.setText(new_name)
                        self.path_edit.setText(dest_path)
                        self.path = dest_path
                        self.update_preview()
                        return True
                    except shutil.SameFileError:
                        # If the file is the same, just use the existing one
                        self.path_edit.setText(file_path)
                        self.path = file_path
                        self.update_preview()
                        return True
            else:
                # File is already in the images directory
                # Extract just the filename for the name field
                base_name = os.path.splitext(os.path.basename(file_path))[0]
                self.name_edit.setText(base_name)
                self.path_edit.setText(file_path)
                self.path = file_path
                self.update_preview()
                return True
        return False
    
    def update_preview(self):
        if os.path.exists(self.path):
            pixmap = QPixmap(self.path)
            if not pixmap.isNull():
                self.preview_label.setPixmap(
                    pixmap.scaled(self.preview_label.size(), 
                                 Qt.AspectRatioMode.KeepAspectRatio,
                                 Qt.TransformationMode.SmoothTransformation))
                return
        
        # Show placeholder if no image
        self.preview_label.setText("No preview")
    
    def get_data(self) -> dict:
        return {
            "name": self.name_edit.text(),
            "path": self.path_edit.text()
        }

class ActionEditor(QWidget):
    """Widget to edit a single action in a step."""
    def __init__(self, action_data: Optional[dict] = None, parent=None):
        super().__init__(parent)
        self.action_data = action_data or {"type": "click"}
        self.templates = []
        # Try to inherit templates from parent StepEditor
        try:
            p = self.parent()
            while p is not None and p.__class__.__name__ != 'StepEditor':
                p = p.parent()
            if p is not None and hasattr(p, 'templates'):
                self.templates = list(getattr(p, 'templates') or [])
        except Exception:
            pass
        # Track else action editors
        self.else_action_widgets = []
        self.init_ui()
    
    def init_ui(self):
        layout = QHBoxLayout()
        
        # Action type
        self.type_combo = QComboBox()
        # Add action types directly as strings to match ActionType enum values
        action_types = ["click", "move", "move_to", "right_click", "double_click", "type", "key_press", "wait", "scroll", "click_and_hold"]
        self.type_combo.addItems(action_types)
        if "type" in self.action_data:
            action_type = self.action_data["type"]
            self.type_combo.setCurrentText(action_type)
        
        # Action parameters
        self.params_widget = QWidget()
        # Use vertical layout to stack main params and conditional group
        self.params_layout = QVBoxLayout(self.params_widget)
        self.params_layout.setContentsMargins(0, 0, 0, 0)
        try:
            # Allow the params area to grow vertically with its content
            self.params_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        except Exception:
            pass
        
        # Action buttons
        btn_frame = QFrame()
        btn_layout = QHBoxLayout(btn_frame)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(2)
        
        # Move up button
        self.move_up_btn = QPushButton("↑")
        self.move_up_btn.setFixedSize(24, 24)
        self.move_up_btn.setToolTip("Move action up")
        self.move_up_btn.clicked.connect(self.move_up)
        
        # Move down button
        self.move_down_btn = QPushButton("↓")
        self.move_down_btn.setFixedSize(24, 24)
        self.move_down_btn.setToolTip("Move action down")
        self.move_down_btn.clicked.connect(self.move_down)
        
        # Remove button
        remove_btn = QPushButton("×")
        remove_btn.setFixedSize(24, 24)
        remove_btn.setToolTip("Remove action")
        remove_btn.clicked.connect(self.remove_self)
        
        # Style buttons
        for btn in [self.move_up_btn, self.move_down_btn, remove_btn]:
            btn.setStyleSheet("""
                QPushButton {
                    border: 1px solid #3a3a3a;
                    border-radius: 3px;
                    padding: 0px;
                    margin: 0px;
                    min-width: 20px;
                    max-width: 20px;
                    min-height: 20px;
                    max-height: 20px;
                }
                QPushButton:hover {
                    background: #3a3a3a;
                }
            """)
        
        # Add buttons to layout
        btn_layout.addWidget(self.move_up_btn)
        btn_layout.addWidget(self.move_down_btn)
        btn_layout.addWidget(remove_btn)
        
        # Add widgets to main layout
        layout.addWidget(QLabel("Action:"))
        layout.addWidget(self.type_combo, 1)  # Allow type combo to expand
        layout.addWidget(self.params_widget, 2)  # Allow params to expand more
        layout.addWidget(btn_frame)
        
        self.type_combo.currentTextChanged.connect(self.update_params)
        self.update_params()
        
        self.setLayout(layout)
    
    def update_params(self):
        # Clear existing params
        while self.params_layout.count():
            item = self.params_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        
        action_type = self.type_combo.currentText()
        
        # Add appropriate parameter controls based on action type
        main_params = QWidget()
        main_layout = QHBoxLayout(main_params)
        main_layout.setContentsMargins(0,0,0,0)
        if action_type == "click":
            main_layout.addWidget(QLabel("Button:"))
            button_combo = QComboBox()
            button_combo.addItems(["left", "middle", "right"])
            button_combo.setCurrentText(self.action_data.get("button", "left"))
            button_combo.currentTextChanged.connect(
                lambda text, key="button": self.update_action_param(key, text))
            main_layout.addWidget(button_combo)
            
            main_layout.addWidget(QLabel("Clicks:"))
            clicks_spin = QSpinBox()
            clicks_spin.setRange(1, 10)
            clicks_spin.setValue(self.action_data.get("clicks", 1))
            clicks_spin.valueChanged.connect(
                lambda value, key="clicks": self.update_action_param(key, value))
            main_layout.addWidget(clicks_spin)
            
        elif action_type in ["type", "key_press"]:
            if action_type == "type":
                main_layout.addWidget(QLabel("Text: "))
                text_edit = QLineEdit(self.action_data.get("text", ""))
                text_edit.textChanged.connect(
                    lambda text, key="text": self.update_action_param(key, text))
                main_layout.addWidget(text_edit)
            else:
                main_layout.addWidget(QLabel("Key:"))
                key_combo = QComboBox()
                try:
                    keys = list(pyautogui.KEYBOARD_KEYS)
                except Exception:
                    keys = []
                key_combo.addItems(keys)
                saved_key = self.action_data.get("key", "")
                if saved_key and saved_key not in keys:
                    key_combo.addItem(saved_key)
                if saved_key:
                    key_combo.setCurrentText(saved_key)
                key_combo.currentTextChanged.connect(
                    lambda text, key="key": self.update_action_param(key, text))
                main_layout.addWidget(key_combo)
            
        elif action_type == "wait":
            main_layout.addWidget(QLabel("Seconds:"))
            seconds_spin = QDoubleSpinBox()
            seconds_spin.setRange(0.1, 60.0)
            seconds_spin.setValue(self.action_data.get("seconds", 1.0))
            seconds_spin.setSingleStep(0.1)
            seconds_spin.valueChanged.connect(
                lambda value, key="seconds": self.update_action_param(key, value))
            main_layout.addWidget(seconds_spin)
            
        elif action_type in ["move", "move_to"]:
            # Add X coordinate
            main_layout.addWidget(QLabel("X:"))
            x_spin = QSpinBox()
            x_spin.setRange(0, 10000)
            x_spin.setValue(self.action_data.get("x", 0))
            x_spin.valueChanged.connect(
                lambda value, key="x": self.update_action_param(key, value))
            main_layout.addWidget(x_spin)
            
            # Add Y coordinate
            main_layout.addWidget(QLabel("Y:"))
            y_spin = QSpinBox()
            y_spin.setRange(0, 10000)
            y_spin.setValue(self.action_data.get("y", 0))
            y_spin.valueChanged.connect(
                lambda value, key="y": self.update_action_param(key, value))
            main_layout.addWidget(y_spin)

            # Show click preview
            show_click_btn = QPushButton("Show Click")
            show_click_btn.setToolTip("Preview the click/move point on screen")
            show_click_btn.clicked.connect(self.show_click_overlay)
            main_layout.addWidget(show_click_btn)
            
            # Toggle random checkbox
            self.random_checkbox = QCheckBox("Toggle Random")
            self.random_checkbox.setChecked(self.action_data.get("random", False))
            self.random_checkbox.toggled.connect(lambda checked: self.update_action_param("random", checked) or self.update_params())
            main_layout.addWidget(self.random_checkbox)

            # If random is checked, allow region selection
            if self.action_data.get("random", False):
                region_btn = QPushButton("Select Region")
                region_btn.clicked.connect(self.select_random_region)
                main_layout.addWidget(region_btn)
                self.random_region_label = QLabel()
                main_layout.addWidget(self.random_region_label)
                self.update_random_region_label()
                # Show random region preview
                show_rr_btn = QPushButton("Show Region")
                show_rr_btn.setToolTip("Preview the random move region")
                show_rr_btn.clicked.connect(self.show_random_region_overlay)
                main_layout.addWidget(show_rr_btn)
            else:
                # Show info
                info_label = QLabel("Moves to center of detected template")
                main_layout.addWidget(info_label)

            # Duration
            main_layout.addWidget(QLabel("Duration (s):"))
            duration_spin = QDoubleSpinBox()
            duration_spin.setRange(0.0, 10.0)
            duration_spin.setSingleStep(0.1)
            duration_spin.setValue(self.action_data.get("duration", 0.0))
            duration_spin.valueChanged.connect(
                lambda value, key="duration": self.update_action_param(key, value))
            main_layout.addWidget(duration_spin)
        
        elif action_type == "scroll":
            main_layout.addWidget(QLabel("Pixels:"))
            pixels_spin = QSpinBox()
            pixels_spin.setRange(-1000, 1000)
            pixels_spin.setValue(self.action_data.get("pixels", 0))
            pixels_spin.valueChanged.connect(
                lambda value, key="pixels": self.update_action_param(key, value))
            main_layout.addWidget(pixels_spin)
            
        elif action_type == "click_and_hold":
            # Add button selection
            main_layout.addWidget(QLabel("Button:"))
            button_combo = QComboBox()
            button_combo.addItems(["left", "middle", "right"])
            button_combo.setCurrentText(self.action_data.get("button", "left"))
            button_combo.currentTextChanged.connect(
                lambda text, key="button": self.update_action_param(key, text))
            main_layout.addWidget(button_combo)
            
            # Add duration control
            main_layout.addWidget(QLabel("Hold for (s):"))
            duration_spin = QDoubleSpinBox()
            duration_spin.setRange(0.1, 60.0)
            duration_spin.setSingleStep(0.1)
            duration_spin.setValue(self.action_data.get("duration", 1.0))
            duration_spin.valueChanged.connect(
                lambda value, key="duration": self.update_action_param(key, value))
            main_layout.addWidget(duration_spin)

        # Add the main params row
        self.params_layout.addWidget(main_params)

        # Conditional IF/ELSE controls
        try:
            cond_group = QGroupBox("Condition")
            try:
                # Prevent the condition group from collapsing when more actions exist
                cond_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.MinimumExpanding)
                cond_group.setMinimumHeight(100)
            except Exception:
                pass
            cond_layout = QVBoxLayout(cond_group)
            cond_layout.setContentsMargins(6,6,6,6)
            # IF row
            if_row = QHBoxLayout()
            if_row.addWidget(QLabel("If Template:"))
            self.if_combo = QComboBox()
            self.if_combo.addItems([""] + list(self.templates))
            # Preload saved value
            self.if_combo.setCurrentText(self.action_data.get("if_template", ""))
            self.if_combo.currentTextChanged.connect(lambda val: self.update_action_param("if_template", val))
            if_row.addWidget(self.if_combo, 1)
            cond_layout.addLayout(if_row)

            # IF region selection row
            if_region_row = QHBoxLayout()
            self.if_region_btn = QPushButton("Set IF Region")
            self.if_region_btn.setToolTip("Limit condition template search to a selected region")
            self.if_region_btn.clicked.connect(self.select_if_region)
            if_region_row.addWidget(self.if_region_btn)
            self.if_region_label = QLabel("IF Region: (none)")
            self.update_if_region_label()
            if_region_row.addWidget(self.if_region_label, 1)
            # Optional clear button
            clear_if_btn = QPushButton("Clear")
            clear_if_btn.setToolTip("Clear IF search region")
            clear_if_btn.clicked.connect(lambda: (self.action_data.pop('if_region', None), self.update_if_region_label()))
            if_region_row.addWidget(clear_if_btn)
            # Optional: show IF region overlay
            show_if_btn = QPushButton("Show IF Region")
            show_if_btn.setToolTip("Preview the IF template search region")
            show_if_btn.clicked.connect(self.show_if_region_overlay)
            if_region_row.addWidget(show_if_btn)
            cond_layout.addLayout(if_region_row)

            # Compact Else Actions header only (count + pop-out button)
            else_header_row = QHBoxLayout()
            try:
                count = len(self.action_data.get('else_actions', []) or [])
            except Exception:
                count = 0
            self.else_count_label = QLabel(f"Else Actions: {count}")
            else_header_row.addWidget(self.else_count_label, 0)
            edit_else_btn = QPushButton("Edit Else Actions…")
            edit_else_btn.setToolTip("Open a window to edit Else Actions")
            edit_else_btn.clicked.connect(self.open_else_actions_dialog)
            else_header_row.addWidget(edit_else_btn, 0)
            cond_layout.addLayout(else_header_row)

            # If-Not Actions header (count + pop-out button)
            ifnot_header_row = QHBoxLayout()
            try:
                not_count = len(self.action_data.get('if_not_actions', []) or [])
            except Exception:
                not_count = 0
            self.ifnot_count_label = QLabel(f"If-Not Actions: {not_count}")
            ifnot_header_row.addWidget(self.ifnot_count_label, 0)
            edit_ifnot_btn = QPushButton("Edit If-Not Actions…")
            edit_ifnot_btn.setToolTip("Open a window to edit actions when IF template is not found")
            edit_ifnot_btn.clicked.connect(self.open_if_not_actions_dialog)
            ifnot_header_row.addWidget(edit_ifnot_btn, 0)
            cond_layout.addLayout(ifnot_header_row)
            self.params_layout.addWidget(cond_group)
        except Exception:
            pass
    
    def update_action_param(self, key: str, value: Any):
        self.action_data[key] = value
    
    def get_action_data(self) -> dict:
        action_type = self.type_combo.currentText()
        self.action_data["type"] = action_type
        # Collect else actions if any
        try:
            if hasattr(self, 'else_action_widgets') and isinstance(self.else_action_widgets, list) and len(self.else_action_widgets) > 0:
                ea_list = []
                for w in self.else_action_widgets:
                    try:
                        ea_list.append(w.get_action_data())
                    except Exception:
                        pass
                if ea_list:
                    self.action_data['else_actions'] = ea_list
            # If no inline editors are present, preserve existing else_actions
        except Exception:
            pass
        return self.action_data

    def refresh_else_actions_inline(self):
        """Update Else Actions count label to reflect current action_data."""
        try:
            count = len(self.action_data.get('else_actions', []) or [])
            if hasattr(self, 'else_count_label') and self.else_count_label is not None:
                self.else_count_label.setText(f"Else Actions: {count}")
            # Also update If-Not Actions count label if present
            not_count = len(self.action_data.get('if_not_actions', []) or [])
            if hasattr(self, 'ifnot_count_label') and self.ifnot_count_label is not None:
                self.ifnot_count_label.setText(f"If-Not Actions: {not_count}")
        except Exception:
            pass

    def open_else_actions_dialog(self):
        """Open a dialog window to edit Else Actions for this action."""
        try:
            dialog = QDialog(self)
            dialog.setWindowTitle("Edit Else Actions")
            layout = QVBoxLayout(dialog)
            layout.setContentsMargins(8,8,8,8)
            layout.setSpacing(8)

            # Container and scroll for dialog actions
            dlg_container = QWidget()
            dlg_v = QVBoxLayout(dlg_container)
            dlg_v.setContentsMargins(6,6,6,6)
            dlg_v.setSpacing(6)

            # Local list of editors
            local_widgets = []
            for ea in list(self.action_data.get('else_actions', []) or []):
                try:
                    w = MiniActionEditor(ea, parent=self)
                    local_widgets.append(w)
                    dlg_v.addWidget(w)
                except Exception:
                    pass
            # Add button
            add_btn = QPushButton("Add Else Action")
            def _add_local():
                w = MiniActionEditor({"type":"click"}, parent=self)
                local_widgets.append(w)
                dlg_v.addWidget(w)
                try:
                    sb = dlg_scroll.verticalScrollBar()
                    if sb is not None:
                        sb.setValue(sb.maximum())
                except Exception:
                    pass
            add_btn.clicked.connect(_add_local)
            layout.addWidget(add_btn)

            dlg_scroll = QScrollArea()
            dlg_scroll.setWidgetResizable(True)
            dlg_scroll.setWidget(dlg_container)
            try:
                dlg_scroll.setVerticalScrollBarPolicy(_Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
                dlg_scroll.setHorizontalScrollBarPolicy(_Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
                dlg_scroll.setFocusPolicy(_Qt.FocusPolicy.StrongFocus)
            except Exception:
                pass
            dlg_scroll.setMinimumHeight(260)
            layout.addWidget(dlg_scroll)

            # Button box
            button_box = QDialogButtonBox(
                QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel,
                parent=dialog
            )
            layout.addWidget(button_box)

            def on_save():
                new_list = []
                for w in local_widgets:
                    try:
                        new_list.append(w.get_action_data())
                    except Exception:
                        pass
                self.action_data['else_actions'] = new_list
                # Update inline UI to reflect changes
                self.refresh_else_actions_inline()
                dialog.accept()
            def on_cancel():
                dialog.reject()
            try:
                button_box.accepted.connect(on_save)
                button_box.rejected.connect(on_cancel)
            except Exception:
                pass

            dialog.exec()
        except Exception:
            pass

    def open_if_not_actions_dialog(self):
        """Open a dialog window to edit actions when IF template is NOT found."""
        try:
            dialog = QDialog(self)
            dialog.setWindowTitle("Edit If-Not Actions")
            layout = QVBoxLayout(dialog)
            layout.setContentsMargins(8,8,8,8)
            layout.setSpacing(8)

            dlg_container = QWidget()
            dlg_v = QVBoxLayout(dlg_container)
            dlg_v.setContentsMargins(6,6,6,6)
            dlg_v.setSpacing(6)

            local_widgets = []
            for na in list(self.action_data.get('if_not_actions', []) or []):
                try:
                    w = MiniActionEditor(na, parent=self)
                    local_widgets.append(w)
                    dlg_v.addWidget(w)
                except Exception:
                    pass

            add_btn = QPushButton("Add If-Not Action")
            def _add_local():
                w = MiniActionEditor({"type":"click"}, parent=self)
                local_widgets.append(w)
                dlg_v.addWidget(w)
                try:
                    sb = dlg_scroll.verticalScrollBar()
                    if sb is not None:
                        sb.setValue(sb.maximum())
                except Exception:
                    pass
            add_btn.clicked.connect(_add_local)
            layout.addWidget(add_btn)

            dlg_scroll = QScrollArea()
            dlg_scroll.setWidgetResizable(True)
            dlg_scroll.setWidget(dlg_container)
            try:
                dlg_scroll.setVerticalScrollBarPolicy(_Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
                dlg_scroll.setHorizontalScrollBarPolicy(_Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
                dlg_scroll.setFocusPolicy(_Qt.FocusPolicy.StrongFocus)
            except Exception:
                pass
            dlg_scroll.setMinimumHeight(260)
            layout.addWidget(dlg_scroll)

            button_box = QDialogButtonBox(
                QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel,
                parent=dialog
            )
            layout.addWidget(button_box)

            def on_save():
                new_list = []
                for w in local_widgets:
                    try:
                        new_list.append(w.get_action_data())
                    except Exception:
                        pass
                self.action_data['if_not_actions'] = new_list
                self.refresh_else_actions_inline()
                dialog.accept()
            def on_cancel():
                dialog.reject()
            try:
                button_box.accepted.connect(on_save)
                button_box.rejected.connect(on_cancel)
            except Exception:
                pass

            dialog.exec()
        except Exception:
            pass
    
    def move_up(self):
        """Move this action up in the list."""
        parent = self.parent()
        while parent is not None:
            if hasattr(parent, 'move_action_up'):
                parent.move_action_up(self)
                break
            parent = parent.parent()
    
    def move_down(self):
        """Move this action down in the list."""
        parent = self.parent()
        while parent is not None:
            if hasattr(parent, 'move_action_down'):
                parent.move_action_down(self)
                break
            parent = parent.parent()
    
    def remove_self(self):
        """Remove this action from its parent."""
        parent = self.parent()
        while parent is not None:
            if hasattr(parent, 'remove_action'):
                parent.remove_action(self)
                break
            parent = parent.parent()

    def select_region(self):
        dialog = ScreenCaptureDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            rect = dialog.get_capture_rect()
            if rect and rect.isValid():
                self.action_data["region"] = [rect.x(), rect.y(), rect.width(), rect.height()]
                self.update_region_label()
            else:
                self.action_data.pop("region", None)
                self.update_region_label()

    def update_region_label(self):
        region = self.action_data.get("region")
        if region and len(region) == 4:
            x, y, w, h = region
            self.region_label.setText(f"Region: ({x}, {y}, {w}x{h})")
        else:
            self.region_label.setText("Region: None (random click disabled)")
    
    def select_random_region(self):
        dialog = ScreenCaptureDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            rect = dialog.get_capture_rect()
            if rect and rect.isValid():
                self.action_data["random_region"] = [rect.x(), rect.y(), rect.width(), rect.height()]
                self.update_random_region_label()
            else:
                self.action_data.pop("random_region", None)
                self.update_random_region_label()

    def update_random_region_label(self):
        region = self.action_data.get("random_region")
        if region and len(region) == 4:
            x, y, w, h = region
            self.random_region_label.setText(f"Region: ({x}, {y}, {w}x{h})")
        else:
            self.random_region_label.setText("Region: None (random move disabled)")

    def select_if_region(self):
        """Capture and set if_region for conditional template search."""
        try:
            dialog = ScreenCaptureDialog(self)
            if dialog.exec() == QDialog.DialogCode.Accepted:
                rect = dialog.get_capture_rect()
                if rect and rect.isValid():
                    self.action_data['if_region'] = [rect.x(), rect.y(), rect.width(), rect.height()]
                else:
                    self.action_data.pop('if_region', None)
                self.update_if_region_label()
        except Exception:
            pass

    def update_if_region_label(self):
        try:
            rr = self.action_data.get('if_region')
            if isinstance(rr, (list, tuple)) and len(rr) == 4:
                x, y, w, h = rr
                txt = f"IF Region: ({x}, {y}, {w}x{h})"
            else:
                txt = "IF Region: (none)"
            if hasattr(self, 'if_region_label') and isinstance(self.if_region_label, QLabel):
                self.if_region_label.setText(txt)
        except Exception:
            pass

    def show_random_region_overlay(self):
        """Preview the random_region for move/move_to actions in this editor."""
        try:
            rr = self.action_data.get('random_region')
            rect = None
            if isinstance(rr, (list, tuple)) and len(rr) == 4:
                x, y, w, h = rr
                rect = QRect(int(x), int(y), int(w), int(h))
            dlg = OverlayPreviewWindow(self, rect=rect, point=None, target_geometry=None, duration_ms=1200, message="Random Region")
            try:
                self._overlay_preview = dlg
            except Exception:
                pass
            dlg.show()
        except Exception:
            pass

    def show_click_overlay(self):
        """Preview the click/move point based on X/Y coordinates in this editor."""
        try:
            x = int(self.action_data.get('x', 0))
            y = int(self.action_data.get('y', 0))
            pt = QPoint(x, y)
            dlg = OverlayPreviewWindow(self, rect=None, point=pt, target_geometry=None, duration_ms=1200, message="Click Point")
            try:
                self._overlay_preview = dlg
            except Exception:
                pass
            dlg.show()
        except Exception:
            pass

    def show_if_region_overlay(self):
        """Preview the IF condition region if configured for this action."""
        try:
            rr = self.action_data.get('if_region')
            rect = None
            if isinstance(rr, (list, tuple)) and len(rr) == 4:
                x, y, w, h = rr
                rect = QRect(int(x), int(y), int(w), int(h))
            dlg = OverlayPreviewWindow(self, rect=rect, point=None, target_geometry=None, duration_ms=1200, message="IF Region")
            try:
                self._overlay_preview = dlg
            except Exception:
                pass
            dlg.show()
        except Exception:
            pass

class MiniActionEditor(QWidget):
    """Minimal nested action editor used for else_actions lists."""
    def __init__(self, action_data: Optional[dict] = None, parent=None):
        super().__init__(parent)
        self.action_data = action_data or {"type": "click"}
        self.params = {}
        self.init_ui()

    def init_ui(self):
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0,0,0,0)
        self.type_combo = QComboBox()
        self.type_combo.addItems(["click", "move", "move_to", "right_click", "double_click", "type", "key_press", "wait", "scroll", "click_and_hold"])
        self.type_combo.setCurrentText(self.action_data.get("type", "click"))
        lay.addWidget(QLabel("Else:"))
        lay.addWidget(self.type_combo)
        # Container for dynamic params
        self.params_container = QWidget()
        self.params_layout = QHBoxLayout(self.params_container)
        self.params_layout.setContentsMargins(0,0,0,0)
        lay.addWidget(self.params_container, 1)
        # Remove button only (no reordering inside nested)
        rm = QPushButton('×'); rm.setFixedSize(24,24)
        rm.clicked.connect(self.remove_self)
        lay.addWidget(rm)
        # Build initial params and hook type change
        self.rebuild_params(self.type_combo.currentText())
        self.type_combo.currentTextChanged.connect(self.on_type_change)

    def update_param(self, key: str, value: Any):
        self.action_data[key] = value

    def get_action_data(self) -> dict:
        self.action_data['type'] = self.type_combo.currentText()
        return self.action_data

    def remove_self(self):
        parent = self.parent()
        # Remove from container and parent's list
        try:
            if hasattr(parent, 'else_action_widgets'):
                parent.else_action_widgets = [w for w in parent.else_action_widgets if w is not self]
        except Exception:
            pass
        self.setParent(None)
        self.deleteLater()

    def clear_params(self):
        try:
            # Remove all widgets from params layout
            while self.params_layout.count():
                item = self.params_layout.takeAt(0)
                w = item.widget()
                if w:
                    w.deleteLater()
            self.params.clear()
        except Exception:
            self.params = {}

    def rebuild_params(self, t: str):
        self.clear_params()
        # Helper creators bind to params_layout
        def add_label(text):
            self.params_layout.addWidget(QLabel(text))
        def add_spin(label, key, rng=(0,10000), val=0, dble=False):
            add_label(label)
            if dble:
                w = QDoubleSpinBox(); w.setRange(float(rng[0]), float(rng[1])); w.setSingleStep(0.1); w.setValue(float(val))
            else:
                w = QSpinBox(); w.setRange(int(rng[0]), int(rng[1])); w.setValue(int(val))
            w.valueChanged.connect(lambda v, k=key: self.update_param(k, v))
            self.params[key] = w
            self.params_layout.addWidget(w)
        def add_text(label, key, val=""):
            add_label(label)
            w = QLineEdit(str(val)); w.textChanged.connect(lambda tt, k=key: self.update_param(k, tt))
            self.params[key] = w
            self.params_layout.addWidget(w)
        def add_btn_combo():
            add_label("Button:")
            w = QComboBox(); w.addItems(["left","middle","right"]); w.setCurrentText(self.action_data.get("button","left"))
            w.currentTextChanged.connect(lambda tbtn: self.update_param("button", tbtn))
            self.params['button'] = w
            self.params_layout.addWidget(w)
        # Build minimal param set based on type
        if t == 'click':
            add_btn_combo(); add_spin('Clicks:', 'clicks', (1,10), self.action_data.get('clicks',1))
        elif t in ('move','move_to'):
            add_spin('X:', 'x', (0,10000), self.action_data.get('x',0))
            add_spin('Y:', 'y', (0,10000), self.action_data.get('y',0))
            add_spin('Duration:', 'duration', (0,10), self.action_data.get('duration',0.0), dble=True)
            # Random toggle and region for move_to
            if t == 'move_to':
                # Random checkbox
                add_label('Random:')
                rand_cb = QCheckBox()
                rand_cb.setChecked(bool(self.action_data.get('random', False)))
                rand_cb.stateChanged.connect(lambda s: self.update_param('random', bool(s)))
                self.params['random'] = rand_cb
                self.params_layout.addWidget(rand_cb)
                # Set Random Region button
                btn = QPushButton('Set Random Region')
                btn.clicked.connect(self.select_random_region)
                self.params_layout.addWidget(btn)
                # Readout label
                self.random_readout = QLabel()
                self.update_random_region_label()
                self.params_layout.addWidget(self.random_readout)
        elif t == 'wait':
            add_spin('Seconds:', 'seconds', (0,60), self.action_data.get('seconds',1.0), dble=True)
        elif t == 'scroll':
            add_spin('Pixels:', 'pixels', (-1000,1000), self.action_data.get('pixels',0))
        elif t in ('type','key_press'):
            if t == 'type':
                add_text('Text:', 'text', self.action_data.get('text',''))
            else:
                add_label('Key:')
                w = QComboBox()
                try:
                    keys = list(pyautogui.KEYBOARD_KEYS)
                except Exception:
                    keys = []
                w.addItems(keys)
                saved = self.action_data.get('key','')
                if saved and saved not in keys:
                    w.addItem(saved)
                if saved:
                    w.setCurrentText(saved)
                w.currentTextChanged.connect(lambda tt: self.update_param('key', tt))
                self.params['key'] = w
                self.params_layout.addWidget(w)
        else:
            # No extra params for right_click/double_click/click_and_hold here
            pass

    def on_type_change(self, new_type: str):
        # Update stored type and refresh fields
        self.action_data['type'] = new_type
        self.rebuild_params(new_type)

    def select_random_region(self):
        """Capture and set a random_region for this nested else action."""
        try:
            dialog = ScreenCaptureDialog(self)
            if dialog.exec() == QDialog.DialogCode.Accepted:
                rect = dialog.get_capture_rect()
                if rect and rect.isValid():
                    self.action_data['random_region'] = [rect.x(), rect.y(), rect.width(), rect.height()]
                else:
                    self.action_data.pop('random_region', None)
                self.update_random_region_label()
        except Exception:
            pass

    def update_random_region_label(self):
        try:
            rr = self.action_data.get('random_region')
            if isinstance(rr, (list, tuple)) and len(rr) == 4:
                x, y, w, h = rr
                lbl = f"Random Region: ({x}, {y}, {w}x{h})"
            else:
                lbl = "Random Region: (none)"
            if hasattr(self, 'random_readout') and isinstance(self.random_readout, QLabel):
                self.random_readout.setText(lbl)
        except Exception:
            pass

    def show_random_region_overlay(self):
        """Preview the random_region for move_to actions."""
        try:
            rr = self.action_data.get('random_region')
            rect = None
            if isinstance(rr, (list, tuple)) and len(rr) == 4:
                x, y, w, h = rr
                rect = QRect(int(x), int(y), int(w), int(h))
            dlg = OverlayPreviewWindow(self, rect=rect, point=None, target_geometry=None, duration_ms=1200, message="Random Region")
            try:
                self._overlay_preview = dlg
            except Exception:
                pass
            dlg.show()
        except Exception:
            pass

    def show_click_overlay(self):
        """Preview the click/move point for actions with X/Y coordinates."""
        try:
            x = int(self.action_data.get('x', 0))
            y = int(self.action_data.get('y', 0))
            pt = QPoint(x, y)
            dlg = OverlayPreviewWindow(self, rect=None, point=pt, target_geometry=None, duration_ms=1200, message="Click Point")
            try:
                self._overlay_preview = dlg
            except Exception:
                pass
            dlg.show()
        except Exception:
            pass

    def show_if_region_overlay(self):
        """Preview the IF condition region if set."""
        try:
            rr = self.action_data.get('if_region')
            rect = None
            if isinstance(rr, (list, tuple)) and len(rr) == 4:
                x, y, w, h = rr
                rect = QRect(int(x), int(y), int(w), int(h))
            dlg = OverlayPreviewWindow(self, rect=rect, point=None, target_geometry=None, duration_ms=1200, message="IF Region")
            try:
                self._overlay_preview = dlg
            except Exception:
                pass
            dlg.show()
        except Exception:
            pass

class StepEditor(QGroupBox):
    """Widget to edit a single step in a sequence."""
    def __init__(self, step_data: Optional[dict] = None, templates: List[str] = None, parent=None):
        super().__init__(parent)
        self.step_data = step_data or {
            "find": "",
            "required": True,
            "timeout": 10,
            "actions": [{"type": "click"}]
        }
        self.templates = templates or []
        self.action_widgets = []
        self.parent_sequence = parent  # Store reference to parent sequence
        self.search_region = self.step_data.get('search_region')  # Initialize search_region from step_data
        self.init_ui()
    
    def init_ui(self):
        # Main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Step header with remove button
        header_layout = QHBoxLayout()
        self.step_label = QLabel("Step")
        
        # Remove step button
        self.remove_btn = QPushButton("×")
        self.remove_btn.setFixedSize(24, 24)
        self.remove_btn.setStyleSheet("""
            QPushButton {
                border: 1px solid #ff6b6b;
                border-radius: 12px;
                color: #ff6b6b;
                font-weight: bold;
                background: transparent;
                padding: 0px;
                margin: 0px;
            }
            QPushButton:hover {
                background: #ff6b6b;
                color: white;
            }
        """)
        self.remove_btn.clicked.connect(self.remove_self)

        # Move step up/down buttons
        self.up_btn = QPushButton("↑")
        self.up_btn.setFixedSize(24, 24)
        self.up_btn.setToolTip("Move step up")
        self.down_btn = QPushButton("↓")
        self.down_btn.setFixedSize(24, 24)
        self.down_btn.setToolTip("Move step down")
        # Connect to parent sequence editor
        try:
            if hasattr(self, 'parent_sequence') and self.parent_sequence is not None:
                self.up_btn.clicked.connect(lambda: getattr(self.parent_sequence, 'move_step_up')(self))
                self.down_btn.clicked.connect(lambda: getattr(self.parent_sequence, 'move_step_down')(self))
        except Exception:
            pass
        
        header_layout.addWidget(self.step_label)
        header_layout.addStretch()
        header_layout.addWidget(self.up_btn)
        header_layout.addWidget(self.down_btn)
        header_layout.addWidget(self.remove_btn)
        
        # Template selection and region group
        template_group = QGroupBox("Template")
        template_layout = QVBoxLayout(template_group)
        
        # First row: Template selection
        template_select_layout = QHBoxLayout()
        
        self.template_combo = QComboBox()
        self.template_combo.addItems([""] + self.templates)
        # Per-step monitor selection
        self.monitor_combo = QComboBox()
        self.monitor_combo.setToolTip("Optional monitor override for this step")
        self.monitor_combo.addItem("Use Global", None)
        self.monitor_combo.addItem("All Monitors", "ALL")
        try:
            screens = QGuiApplication.screens()
            for idx, screen in enumerate(screens, start=1):
                geom = screen.geometry()
                label = f"Screen {idx} ({geom.x()},{geom.y()},{geom.width()}x{geom.height()})"
                self.monitor_combo.addItem(label, (geom.x(), geom.y(), geom.width(), geom.height()))
        except Exception:
            pass
        # Preselect from step_data if present
        if "monitor" in self.step_data:
            data = self.step_data.get("monitor")
            # Normalize list to tuple for comparison when loaded from JSON
            if isinstance(data, list) and len(data) == 4:
                data = tuple(data)
            # Find matching data
            for i in range(self.monitor_combo.count()):
                if self.monitor_combo.itemData(i) == data:
                    self.monitor_combo.setCurrentIndex(i)
                    break
        
        # Add region selection button
        self.select_region_btn = QPushButton("Select Search Region")
        self.select_region_btn.clicked.connect(self.select_search_region)
        # Show region preview button
        self.show_region_btn = QPushButton("Show Region")
        self.show_region_btn.setToolTip("Preview the current search region on screen")
        self.show_region_btn.clicked.connect(self.show_search_region_overlay)
        
        template_select_layout.addWidget(QLabel("Find:"))
        template_select_layout.addWidget(self.template_combo, 1)  # Allow combo box to expand
        template_select_layout.addWidget(QLabel("Monitor:"))
        template_select_layout.addWidget(self.monitor_combo)
        template_select_layout.addWidget(self.select_region_btn)
        template_select_layout.addWidget(self.show_region_btn)
        
        # Region status label
        self.region_status = QLabel("Region: Full Screen")
        self.update_region_status()
        
        # Add to template layout
        template_layout.addLayout(template_select_layout)
        template_layout.addWidget(self.region_status)
        
        # Set initial values
        if "find" in self.step_data:
            index = self.template_combo.findText(self.step_data["find"])
            if index >= 0:
                self.template_combo.setCurrentIndex(index)
                
        # Set initial search region if it exists
        if "search_region" in self.step_data:
            self.search_region = self.step_data["search_region"]
            self.update_region_status()
        
        # Second row: Options
        options_layout = QHBoxLayout()
        
        self.required_check = QCheckBox("Required")
        self.required_check.setChecked(self.step_data.get("required", True))
        options_layout.addWidget(self.required_check)
        
        # Confidence setting
        options_layout.addWidget(QLabel("Confidence:"))
        self.confidence_spin = QDoubleSpinBox()
        self.confidence_spin.setRange(0.1, 1.0)
        self.confidence_spin.setSingleStep(0.05)
        self.confidence_spin.setValue(self.step_data.get("confidence", 0.9))
        self.confidence_spin.setDecimals(2)
        options_layout.addWidget(self.confidence_spin)
        
        # Timeout setting
        options_layout.addWidget(QLabel("Timeout:"))
        self.timeout_spin = QSpinBox()
        self.timeout_spin.setRange(1, 300)
        self.timeout_spin.setValue(self.step_data.get("timeout", 10))
        self.timeout_spin.setSuffix(" sec")
        options_layout.addWidget(self.timeout_spin)
        
        # Per-step loop count
        options_layout.addWidget(QLabel("Step Loops:"))
        self.step_loop_spin = QSpinBox()
        self.step_loop_spin.setRange(1, 9999)
        self.step_loop_spin.setValue(self.step_data.get("loop_count", 1))
        options_layout.addWidget(self.step_loop_spin)
        
        options_layout.addStretch()
        template_layout.addLayout(options_layout)

        # Detection strategy and advanced thresholds
        strategy_layout = QHBoxLayout()
        strategy_layout.addWidget(QLabel("Detection Strategy:"))
        self.strategy_combo = QComboBox()
        # Display labels with underlying data values
        self.strategy_combo.addItem("Default (template)", None)
        self.strategy_combo.addItem("Feature (scale/rotation)", "feature")
        # Preselect from step_data
        try:
            strategy_val = self.step_data.get("strategy")
            idx = self.strategy_combo.findData(strategy_val)
            if idx >= 0:
                self.strategy_combo.setCurrentIndex(idx)
        except Exception:
            pass
        strategy_layout.addWidget(self.strategy_combo)
        # Advanced params
        self.min_inliers_spin = QSpinBox()
        self.min_inliers_spin.setRange(4, 1000)
        self.min_inliers_spin.setValue(int(self.step_data.get("min_inliers", 12)))
        strategy_layout.addWidget(QLabel("Min Inliers:"))
        strategy_layout.addWidget(self.min_inliers_spin)
        self.ratio_spin = QDoubleSpinBox()
        self.ratio_spin.setRange(0.1, 0.99)
        self.ratio_spin.setSingleStep(0.05)
        self.ratio_spin.setDecimals(2)
        self.ratio_spin.setValue(float(self.step_data.get("ratio_thresh", 0.75)))
        strategy_layout.addWidget(QLabel("Ratio:"))
        strategy_layout.addWidget(self.ratio_spin)
        self.ransac_spin = QDoubleSpinBox()
        self.ransac_spin.setRange(0.5, 20.0)
        self.ransac_spin.setSingleStep(0.5)
        self.ransac_spin.setDecimals(1)
        self.ransac_spin.setValue(float(self.step_data.get("ransac_thresh", 4.0)))
        strategy_layout.addWidget(QLabel("RANSAC:"))
        strategy_layout.addWidget(self.ransac_spin)
        strategy_layout.addStretch()
        template_layout.addLayout(strategy_layout)

        def toggle_advanced_by_strategy():
            use_feature = (self.strategy_combo.currentData() == "feature")
            self.min_inliers_spin.setEnabled(use_feature)
            self.ratio_spin.setEnabled(use_feature)
            self.ransac_spin.setEnabled(use_feature)
        self.strategy_combo.currentIndexChanged.connect(toggle_advanced_by_strategy)
        toggle_advanced_by_strategy()

        # Wire change events to persist config and refresh UI
        def notify_change():
            try:
                # Walk up to the MainWindow
                p = self.parent()
                while p is not None and not isinstance(p, MainWindow):
                    p = p.parent()
                # Skip during initial load
                if isinstance(p, MainWindow) and not getattr(p, '_suppress_auto_save', False):
                    p.update_config_from_ui()
                    p.save_config()
            except Exception:
                pass

        try:
            self.template_combo.currentTextChanged.connect(lambda _=None: notify_change())
            self.required_check.toggled.connect(lambda _=None: notify_change())
            self.confidence_spin.valueChanged.connect(lambda _=None: notify_change())
            self.timeout_spin.valueChanged.connect(lambda _=None: notify_change())
            self.step_loop_spin.valueChanged.connect(lambda _=None: notify_change())
            self.strategy_combo.currentIndexChanged.connect(lambda _=None: notify_change())
            self.min_inliers_spin.valueChanged.connect(lambda _=None: notify_change())
            self.ratio_spin.valueChanged.connect(lambda _=None: notify_change())
            self.ransac_spin.valueChanged.connect(lambda _=None: notify_change())
            self.monitor_combo.currentIndexChanged.connect(lambda _=None: notify_change())
        except Exception:
            pass
        
        # Actions group with scrollable area
        actions_group = QGroupBox("Actions")
        actions_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        
        # Main layout for the group box
        actions_main_layout = QVBoxLayout(actions_group)
        actions_main_layout.setContentsMargins(5, 15, 5, 5)
        actions_main_layout.setSpacing(5)
        
        # Scroll area for actions
        self.actions_scroll = QScrollArea()
        self.actions_scroll.setWidgetResizable(True)
        self.actions_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.actions_scroll.setMinimumHeight(200)  # Ensure visibility
        self.actions_scroll.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        try:
            from PyQt6.QtCore import Qt as _Qt
            self.actions_scroll.setVerticalScrollBarPolicy(_Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        except Exception:
            pass
        
        # Container for action widgets with proper size policies
        self.actions_container = QWidget()
        # Allow vertical growth based on content so scrollbars appear automatically
        self.actions_container.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        
        # Use a vertical layout for the container
        self.actions_layout = QVBoxLayout(self.actions_container)
        self.actions_layout.setSpacing(8)
        self.actions_layout.setContentsMargins(5, 5, 5, 5)
        self.actions_layout.setAlignment(Qt.AlignmentFlag.AlignTop)  # Align items to the top
        
        # Make sure the container doesn't expand vertically more than needed
        self.actions_container.setMinimumHeight(50)  # Minimum height to show something
        
        # Set the container as the widget for the scroll area
        self.actions_scroll.setWidget(self.actions_container)
        
        # Add the scroll area to the main layout with stretch
        actions_main_layout.addWidget(self.actions_scroll, 1)  # Allow the scroll area to expand
        
        # Add action buttons in a frame at the bottom
        btn_frame = QFrame()
        btn_frame.setFrameShape(QFrame.Shape.StyledPanel)
        btn_layout = QHBoxLayout(btn_frame)
        btn_layout.setContentsMargins(5, 5, 5, 5)
        btn_layout.setSpacing(5)
        
        # Add action buttons in a grid layout to prevent them from taking too much horizontal space
        grid_layout = QGridLayout()
        row, col = 0, 0
        max_cols = 3  # Number of buttons per row
        
        for i, action_type in enumerate(ActionType):
            btn = QPushButton(f"Add {action_type.value}")
            btn.clicked.connect(
                lambda checked, at=action_type.value: self.add_action({"type": at}))
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            
            grid_layout.addWidget(btn, row, col)
            col += 1
            if col >= max_cols:
                col = 0
                row += 1
        
        btn_layout.addLayout(grid_layout)
        actions_main_layout.addWidget(btn_frame)
        
        # Add widgets to main layout
        main_layout.addLayout(header_layout)
        main_layout.addWidget(template_group)
        main_layout.addWidget(actions_group, 1)  # Allow actions to expand
        
        # Add existing actions after UI is fully set up
        for action in self.step_data.get("actions", []):
            self.add_action(action)
        # Recalculate geometry to ensure scrollbar visibility when needed
        self.actions_container.updateGeometry()
        self.actions_scroll.updateGeometry()
        
        # Styling
        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid #3a3a3a;
                border-radius: 4px;
                padding: 8px;
                margin-top: 5px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px;
            }
            QScrollArea {
                border: none;
                background: transparent;
            }
            QFrame#action_buttons {
                background: #2d2d2d;
                border-radius: 4px;
            }
        """)
    
    def add_action(self, action_data=None):
        """Add a new action to this step.
        
        Args:
            action_data: Optional dictionary containing action data. If None, creates a default click action.
        """
        if action_data is None:
            action_data = {"type": "click"}
            
        # Create the action editor
        action_editor = ActionEditor(action_data, self)
        try:
            # Allow the editor to grow vertically based on its content
            action_editor.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        except Exception:
            pass
        
        # Add to layout and widget list
        self.actions_layout.addWidget(action_editor)
        self.action_widgets.append(action_editor)
        
        # Ensure the container grows with content so scrollbars appear when needed
        try:
            per_item = 150  # Estimated height per action editor including condition group
            min_height = max(200, len(self.action_widgets) * per_item)
            self.actions_container.setMinimumHeight(min_height)
        except Exception:
            self.actions_container.updateGeometry()
        
        # Ensure the new action is visible
        QTimer.singleShot(50, lambda: self._scroll_widget_into_view(action_editor))

        # Persist config after adding an action
        try:
            p = self.parent()
            while p is not None and not isinstance(p, MainWindow):
                p = p.parent()
            if isinstance(p, MainWindow) and not getattr(p, '_suppress_auto_save', False):
                p.update_config_from_ui()
                p.save_config()
        except Exception:
            pass
        
        return action_editor

    def set_templates(self, templates: List[str]):
        """Update available templates and refresh the template combo while preserving selection."""
        try:
            self.templates = templates or []
            current = self.template_combo.currentText() if hasattr(self, 'template_combo') else ''
            self.template_combo.blockSignals(True)
            self.template_combo.clear()
            self.template_combo.addItems([""] + self.templates)
            self.template_combo.blockSignals(False)
            if current:
                idx = self.template_combo.findText(current)
                if idx >= 0:
                    self.template_combo.setCurrentIndex(idx)
        except Exception:
            pass
    
    def move_action_up(self, action_widget):
        """Move the specified action up in the list."""
        index = self.action_widgets.index(action_widget)
        if index > 0:
            # Remove from current position
            self.actions_layout.removeWidget(action_widget)
            self.action_widgets.pop(index)
            
            # Insert at new position
            new_index = index - 1
            self.actions_layout.insertWidget(new_index, action_widget)
            self.action_widgets.insert(new_index, action_widget)
            
            # Ensure the moved action is visible
            QTimer.singleShot(50, lambda: self._scroll_widget_into_view(action_widget))
    
    def move_action_down(self, action_widget):
        """Move the specified action down in the list."""
        index = self.action_widgets.index(action_widget)
        if index < len(self.action_widgets) - 1:
            # Remove from current position
            self.actions_layout.removeWidget(action_widget)
            self.action_widgets.pop(index)
            
            # Insert at new position
            new_index = index + 1
            self.actions_layout.insertWidget(new_index, action_widget)
            self.action_widgets.insert(new_index, action_widget)
            
            # Ensure the moved action is visible
            QTimer.singleShot(50, lambda: self._scroll_widget_into_view(action_widget))

    def _scroll_widget_into_view(self, widget):
        """Safely scroll the actions area to make the given widget visible.
        Works even if ensureWidgetVisible is unavailable or the widget hasn't fully laid out yet.
        """
        try:
            if not hasattr(self, 'actions_scroll') or self.actions_scroll is None:
                return
            sa = self.actions_scroll
            container = sa.widget()
            if container is None or widget is None:
                return
            # Map widget position to the scroll area's content widget
            pos = widget.mapTo(container, QPoint(0, 0))
            margin = 24
            # Scroll vertically to widget's Y position minus a small margin
            y = max(0, pos.y() - margin)
            sb = sa.verticalScrollBar()
            if sb is not None:
                sb.setValue(y)
        except Exception as e:
            try:
                logger.debug(f"_scroll_widget_into_view failed: {e}")
            except Exception:
                pass
    
    def remove_action(self, action_widget):
        """Remove the specified action from this step."""
        if action_widget in self.action_widgets:
            # Remove from layout
            self.actions_layout.removeWidget(action_widget)
            # Remove from list
            self.action_widgets.remove(action_widget)
            # Delete the widget
            action_widget.setParent(None)
            action_widget.deleteLater()

            # Update minimum height based on remaining actions
            try:
                per_item = 150
                min_height = max(200, len(self.action_widgets) * per_item)
                self.actions_container.setMinimumHeight(min_height)
            except Exception:
                self.actions_container.updateGeometry()

            # Persist config after removing an action
            try:
                p = self.parent()
                while p is not None and not isinstance(p, MainWindow):
                    p = p.parent()
                if isinstance(p, MainWindow) and not getattr(p, '_suppress_auto_save', False):
                    p.update_config_from_ui()
                    p.save_config()
            except Exception:
                pass
    
    def remove_self(self):
        """Remove this step from its parent."""
        try:
            # Log attempt to remove for debugging
            if hasattr(self, 'parent_sequence') and self.parent_sequence and hasattr(self.parent_sequence, 'step_widgets'):
                try:
                    idx = self.parent_sequence.step_widgets.index(self)
                    logger.info(f"Removing failsafe step at index {idx}")
                except ValueError:
                    logger.info("Removing failsafe step (not found in step_widgets list)")
        except Exception as e:
            logger.debug(f"Remove step: logging index failed: {e}")
        # Prefer the explicit parent_sequence reference to avoid QWidget parent chain issues
        if hasattr(self, 'parent_sequence') and self.parent_sequence and hasattr(self.parent_sequence, 'remove_step'):
            self.parent_sequence.remove_step(self)
            return
        # Fallback: walk up the QWidget parent chain to find a container with remove_step
        parent = self.parent()
        while parent is not None:
            if hasattr(parent, 'remove_step'):
                parent.remove_step(self)
                break
            parent = parent.parent()
    
    def select_search_region(self):
        """Open a dialog to select a search region for this step."""
        # Restrict overlay to chosen monitor if set
        target_geometry = None
        try:
            data = self.monitor_combo.currentData() if hasattr(self, 'monitor_combo') else None
            if isinstance(data, tuple) and len(data) == 4:
                x, y, w, h = data
                target_geometry = QRect(x, y, w, h)
        except Exception:
            pass
        dialog = ScreenCaptureDialog(self, target_geometry=target_geometry)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            rect = dialog.get_capture_rect()
            if rect and rect.width() > 0 and rect.height() > 0:
                self.search_region = (rect.x(), rect.y(), rect.width(), rect.height())
                self.update_region_status()
    
    def update_region_status(self):
        """Update the region status label."""
        if self.search_region:
            x, y, w, h = self.search_region
            self.region_status.setText(f"Search Region: ({x}, {y}, {w}x{h})")
        else:
            self.region_status.setText("Search Region: Full Screen")

    def show_search_region_overlay(self):
        """Show a transparent overlay previewing the current search region."""
        try:
            # Determine monitor coverage
            target_geometry = None
            try:
                data = self.monitor_combo.currentData() if hasattr(self, 'monitor_combo') else None
                if isinstance(data, tuple) and len(data) == 4:
                    x, y, w, h = data
                    target_geometry = QRect(x, y, w, h)
                elif data == 'ALL':
                    target_geometry = None  # Full desktop
            except Exception:
                pass

            rect = None
            if getattr(self, 'search_region', None):
                try:
                    x, y, w, h = self.search_region
                    rect = QRect(int(x), int(y), int(w), int(h))
                except Exception:
                    rect = None
            else:
                # If no specific region, draw the selected monitor bounds if available
                rect = target_geometry if isinstance(target_geometry, QRect) and target_geometry.isValid() else None

            dlg = OverlayPreviewWindow(self, rect=rect, point=None, target_geometry=target_geometry, duration_ms=1200, message="Search Region")
            try:
                self._overlay_preview = dlg
            except Exception:
                pass
            dlg.show()
        except Exception:
            pass
    
    def get_step_data(self) -> dict:
        step_data = {
            "find": self.template_combo.currentText(),
            "required": self.required_check.isChecked(),
            "confidence": round(self.confidence_spin.value(), 2),  # Round to 2 decimal places
            "timeout": self.timeout_spin.value(),
            "actions": [w.get_action_data() for w in self.action_widgets],
            "loop_count": self.step_loop_spin.value()
        }
        # Include per-step monitor override
        try:
            if hasattr(self, 'monitor_combo'):
                step_data["monitor"] = self.monitor_combo.currentData()
        except Exception:
            pass
        # Include strategy and thresholds when feature strategy is selected
        try:
            strategy_val = self.strategy_combo.currentData() if hasattr(self, 'strategy_combo') else None
            if strategy_val:
                step_data["strategy"] = strategy_val
                step_data["min_inliers"] = int(self.min_inliers_spin.value()) if hasattr(self, 'min_inliers_spin') else 12
                step_data["ratio_thresh"] = float(self.ratio_spin.value()) if hasattr(self, 'ratio_spin') else 0.75
                step_data["ransac_thresh"] = float(self.ransac_spin.value()) if hasattr(self, 'ransac_spin') else 4.0
        except Exception:
            pass
        if self.search_region:
            step_data["search_region"] = self.search_region
        return step_data

class GroupCallStepEditor(QGroupBox):
    """Lightweight editor for a group call step."""
    def __init__(self, step_data: Optional[dict] = None, groups: Optional[list] = None, parent=None):
        super().__init__(parent)
        self.parent_sequence = parent
        self.step_data = step_data or {"call_group": ""}
        self.groups = groups or []
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        header_layout = QHBoxLayout()
        self.step_label = QLabel("Group Call")
        self.remove_btn = QPushButton("×")
        self.remove_btn.setFixedSize(24, 24)
        self.remove_btn.clicked.connect(self.remove_self)
        # Move step up/down buttons
        self.up_btn = QPushButton("↑")
        self.up_btn.setFixedSize(24, 24)
        self.up_btn.setToolTip("Move step up")
        self.down_btn = QPushButton("↓")
        self.down_btn.setFixedSize(24, 24)
        self.down_btn.setToolTip("Move step down")
        try:
            if hasattr(self, 'parent_sequence') and self.parent_sequence is not None:
                self.up_btn.clicked.connect(lambda: getattr(self.parent_sequence, 'move_step_up')(self))
                self.down_btn.clicked.connect(lambda: getattr(self.parent_sequence, 'move_step_down')(self))
        except Exception:
            pass
        header_layout.addWidget(self.step_label)
        header_layout.addStretch()
        header_layout.addWidget(self.up_btn)
        header_layout.addWidget(self.down_btn)
        header_layout.addWidget(self.remove_btn)

        group_layout = QHBoxLayout()
        group_layout.addWidget(QLabel("Group:"))
        self.group_combo = QComboBox()
        self.group_combo.addItems([""] + list(self.groups))
        if "call_group" in self.step_data:
            idx = self.group_combo.findText(self.step_data["call_group"])
            if idx >= 0:
                self.group_combo.setCurrentIndex(idx)
        group_layout.addWidget(self.group_combo, 1)
        # Persist on group selection change
        try:
            def _commit_group_change():
                p = self.parent()
                while p is not None and not isinstance(p, MainWindow):
                    p = p.parent()
                if isinstance(p, MainWindow) and not getattr(p, '_suppress_auto_save', False):
                    p.update_config_from_ui()
                    p.save_config()
            self.group_combo.currentTextChanged.connect(lambda _=None: _commit_group_change())
        except Exception:
            pass

        main_layout.addLayout(header_layout)
        main_layout.addLayout(group_layout)

    def remove_self(self):
        parent = self.parent()
        while parent is not None:
            if hasattr(parent, 'remove_step'):
                parent.remove_step(self)
                break
            parent = parent.parent()

    def get_step_data(self) -> dict:
        return {"call_group": self.group_combo.currentText().strip()}

class SequenceEditor(QGroupBox):
    """Widget to edit a sequence of steps."""
    def __init__(self, sequence_data: Optional[dict] = None, templates: List[str] = None, parent=None, groups: Optional[list] = None):
        super().__init__(parent)
        self.sequence_data = sequence_data or {"name": "New Sequence", "steps": []}
        self.templates = templates or []
        self.groups = groups or []
        self.step_widgets = []
        self._ui_initialized = False
        self.init_ui()
    
    def clear_ui(self):
        """Clear all widgets from the UI."""
        if hasattr(self, 'steps_layout'):
            # Clear all step widgets
            for i in reversed(range(self.steps_layout.count())):
                item = self.steps_layout.itemAt(i)
                if item.widget():
                    item.widget().deleteLater()
                elif item.layout():
                    # If it's a layout, remove all items from it
                    for j in reversed(range(item.layout().count())):
                        subitem = item.layout().itemAt(j)
                        if subitem.widget():
                            subitem.widget().deleteLater()
            self.step_widgets = []
    
    def _add_steps_from_data(self):
        """Add steps from the sequence data to the UI."""
        # Clear existing steps
        self.clear_ui()
        
        # Add steps from sequence data
        for step in self.sequence_data.get("steps", []):
            self.add_step(step)
    
    def init_ui(self):
        # Skip if UI is already initialized
        if hasattr(self, '_ui_initialized') and self._ui_initialized:
            self._add_steps_from_data()
            return
            
        # Main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Sequence header with name and add buttons
        header_layout = QHBoxLayout()
        
        # Sequence name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Name:"))
        self.name_edit = QLineEdit(self.sequence_data.get("name", "New Sequence"))
        name_layout.addWidget(self.name_edit, 1)  # Allow name field to expand
        
        # Add step buttons
        self.add_step_btn = QPushButton("Add Step")
        self.add_step_btn.clicked.connect(self.add_step)
        self.add_group_call_btn = QPushButton("Add Group Call")
        self.add_group_call_btn.clicked.connect(lambda: self.add_step({"call_group": ""}))
        
        # Add to header
        header_layout.addLayout(name_layout)
        header_layout.addWidget(self.add_step_btn, 0, Qt.AlignmentFlag.AlignRight)
        header_layout.addWidget(self.add_group_call_btn, 0, Qt.AlignmentFlag.AlignRight)
        
        # Scroll area for steps
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll_area.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        
        # Container widget for steps
        self.steps_container = QWidget()
        # Prefer vertical growth based on content; allow scroll area to show bars as needed
        self.steps_container.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.steps_layout = QVBoxLayout(self.steps_container)
        self.steps_layout.setSpacing(10)
        self.steps_layout.setContentsMargins(0, 0, 10, 10)
        self.steps_layout.addStretch()  # Add stretch to push steps to top
        
        # Set up the scroll area
        self.scroll_area.setWidget(self.steps_container)
        self.scroll_area.setWidgetResizable(True)
        
        # Add widgets to main layout
        main_layout.addLayout(header_layout)
        main_layout.addWidget(self.scroll_area, 1)  # 1 is stretch factor
        
        self._ui_initialized = True
        
        # Add existing steps
        self._add_steps_from_data()
        # Ensure scroll metrics are recalculated after population
        QTimer.singleShot(0, lambda: self.scroll_area.updateGeometry())
        
        # Styling
        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid #3a3a3a;
                border-radius: 4px;
                margin-top: 1em;
            }
            QScrollArea {
                border: none;
                background: transparent;
            }
            QScrollBar:vertical {
                border: none;
                background: #2d2d2d;
                width: 10px;
                margin: 0px 0px 0px 0px;
            }
            QScrollBar::handle:vertical {
                background: #3e3e3e;
                min-height: 20px;
                border-radius: 5px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
    
    def update_groups(self, groups: Optional[list] = None):
        """Update available groups and refresh any group-call step dropdowns.
        Preserves current selections.
        """
        try:
            self.groups = groups or []
            for sw in getattr(self, 'step_widgets', []):
                if isinstance(sw, GroupCallStepEditor) and hasattr(sw, 'group_combo'):
                    current = sw.group_combo.currentText()
                    sw.group_combo.blockSignals(True)
                    sw.group_combo.clear()
                    sw.group_combo.addItems([""] + list(self.groups))
                    # Restore selection if it exists in the new list
                    idx = sw.group_combo.findText(current)
                    if idx >= 0:
                        sw.group_combo.setCurrentIndex(idx)
                    sw.group_combo.blockSignals(False)
        except Exception:
            pass


    def add_step(self, step_data: Optional[dict] = None):
        """Add a new step to the sequence."""
        try:
            # Ensure step_data has the required structure with at least one default action
            if step_data is None or not isinstance(step_data, dict):
                step_data = {
                    "find": "",
                    "required": True,
                    "timeout": 10,
                    "confidence": 0.8,
                    "detection_strategy": "default",
                    "step_loops": 1,
                    "monitor": None,
                    "actions": [{"type": "click"}]
                }
            # Inject global search_region if available
            try:
                p = self.parent()
                from types import SimpleNamespace
                while p is not None and p.__class__.__name__ != 'MainWindow':
                    p = p.parent()
                if p is not None and hasattr(p, 'search_region') and isinstance(p.search_region, (list, tuple)) and len(p.search_region) == 4:
                    step_data.setdefault('search_region', list(p.search_region))
                elif p is not None and isinstance(getattr(p, 'config', {}), dict):
                    sr = (p.config or {}).get('search_region')
                    if isinstance(sr, list) and len(sr) == 4:
                        step_data.setdefault('search_region', sr)
            except Exception:
                pass
            # If this is a group call, use the GroupCallStepEditor
            if isinstance(step_data, dict) and 'call_group' in step_data:
                # Ensure groups list is current before building the editor
                step_editor = GroupCallStepEditor(step_data, self.groups, self)
            else:
                if not step_data.get('actions') or not isinstance(step_data['actions'], list):
                    step_data['actions'] = [{"type": "click"}]
                step_editor = StepEditor(step_data, self.templates, self)
        except Exception as e:
            print(f"Error creating step: {str(e)}")
            # Fallback to default step data
            step_editor = StepEditor({
                "find": "",
                "required": True,
                "timeout": 10,
                "actions": [{"type": "click"}]
            }, self.templates, self)
        
        # Set size policy for the step editor (allow vertical growth based on content)
        step_editor.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        
        # Insert the new step before the stretch at the end
        self.steps_layout.insertWidget(self.steps_layout.count() - 1, step_editor)
        
        # Add the step widget to our list
        self.step_widgets.append(step_editor)
        
        # Update the step numbers
        self.update_step_numbers()
        
        # Ensure the new step is visible (safe scrolling)
        try:
            import weakref
            ref = weakref.ref(step_editor)
            QTimer.singleShot(100, lambda: self._scroll_step_into_view(ref))
        except Exception:
            QTimer.singleShot(100, lambda: self._scroll_widget_into_view_safe(self.scroll_area, step_editor))
        
        # Update the container size
        self.steps_container.adjustSize()
        
        return step_editor
    
    def remove_step(self, step_editor):
        """Remove a step from the sequence.
        
        Args:
            step_editor: The StepEditor widget to remove
        """
        if step_editor in self.step_widgets:
            try:
                idx = self.step_widgets.index(step_editor)
                logger.info(f"FailsafeSequenceEditor: removing step at index {idx}")
            except Exception:
                logger.info("FailsafeSequenceEditor: removing step (index unknown)")
            # Remove from layout
            self.steps_layout.removeWidget(step_editor)
            
            # Remove from our list
            self.step_widgets.remove(step_editor)
            
            # Delete the widget
            step_editor.setParent(None)
            step_editor.deleteLater()
            
            # Update step numbers
            self.update_step_numbers()
            
            # Update the container size
            self.steps_container.adjustSize()
            self.steps_container.update()
            self.scroll_area.update()
        else:
            # Fallback: if the widget isn't tracked (edge case), remove it anyway
            logger.info("FailsafeSequenceEditor: step not tracked; removing widget directly")
            try:
                self.steps_layout.removeWidget(step_editor)
            except Exception:
                pass
            try:
                step_editor.setParent(None)
                step_editor.deleteLater()
            except Exception:
                pass
            # Try to refresh numbering and layout
            try:
                self.update_step_numbers()
            except Exception:
                pass
            self.steps_container.adjustSize()
            self.steps_container.update()
            self.scroll_area.update()

    def move_step_up(self, step_editor):
        """Move the specified step up in the sequence."""
        if step_editor in self.step_widgets:
            idx = self.step_widgets.index(step_editor)
            if idx > 0:
                # Update layout order
                try:
                    self.steps_layout.removeWidget(step_editor)
                    self.step_widgets.pop(idx)
                    new_idx = idx - 1
                    # Insert before stretch
                    layout_idx = min(new_idx, self.steps_layout.count() - 1)
                    self.steps_layout.insertWidget(layout_idx, step_editor)
                    self.step_widgets.insert(new_idx, step_editor)
                    self.update_step_numbers()
                    try:
                        import weakref
                        ref = weakref.ref(step_editor)
                        QTimer.singleShot(50, lambda: self._scroll_step_into_view(ref))
                    except Exception:
                        QTimer.singleShot(50, lambda: self._scroll_widget_into_view_safe(self.scroll_area, step_editor))
                except Exception:
                    pass

    def move_step_down(self, step_editor):
        """Move the specified step down in the sequence."""
        if step_editor in self.step_widgets:
            idx = self.step_widgets.index(step_editor)
            if idx < len(self.step_widgets) - 1:
                try:
                    self.steps_layout.removeWidget(step_editor)
                    self.step_widgets.pop(idx)
                    new_idx = idx + 1
                    layout_idx = min(new_idx, self.steps_layout.count() - 1)
                    self.steps_layout.insertWidget(layout_idx, step_editor)
                    self.step_widgets.insert(new_idx, step_editor)
                    self.update_step_numbers()
                    try:
                        import weakref
                        ref = weakref.ref(step_editor)
                        QTimer.singleShot(50, lambda: self._scroll_step_into_view(ref))
                    except Exception:
                        QTimer.singleShot(50, lambda: self._scroll_widget_into_view_safe(self.scroll_area, step_editor))
                except Exception:
                    pass
    
    def update_step_numbers(self):
        """Update the step numbers in the UI."""
        for i, step in enumerate(self.step_widgets, 1):
            if hasattr(step, 'step_label'):
                step.step_label.setText(f"Step {i}")

    def _scroll_step_into_view(self, widget_ref):
        """Scroll the sequence scroll area to bring the referenced step editor into view.
        Uses weak references to avoid RuntimeError when the widget is deleted before the timer fires.
        """
        try:
            widget = widget_ref()
            if widget is None:
                return
            self._scroll_widget_into_view_safe(self.scroll_area, widget)
        except Exception as e:
            try:
                logger.debug(f"_scroll_step_into_view failed: {e}")
            except Exception:
                pass

    def _scroll_widget_into_view_safe(self, scroll_area, widget):
        """Generic safe scroll helper for QScrollArea to make a child widget visible."""
        try:
            if scroll_area is None or widget is None:
                return
            container = scroll_area.widget()
            if container is None:
                return
            pos = widget.mapTo(container, QPoint(0, 0))
            margin = 24
            y = max(0, pos.y() - margin)
            sb = scroll_area.verticalScrollBar()
            if sb is not None:
                sb.setValue(y)
        except Exception:
            pass

    def get_sequence_data(self) -> dict:
        """Get the sequence data including all steps and their actions."""
        data = {
            "name": self.name_edit.text(),
            "steps": [w.get_step_data() for w in self.step_widgets]
        }
        return data

class TemplateTester(QWidget):
    """Widget for testing template matching with live preview."""
    def __init__(self, bot, templates, parent=None):
        super().__init__(parent)
        self.bot = bot
        self.templates = templates
        self.template_path = ""
        self.search_region = None  # Store the selected region (x, y, width, height)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_preview)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Template selection
        template_group = QGroupBox("Template")
        template_layout = QVBoxLayout(template_group)
        
        # Template dropdown
        self.template_combo = QComboBox()
        self.template_combo.addItem("Select a template...", None)
        for name in sorted(self.templates.keys()):
            self.template_combo.addItem(name, self.templates[name])
        self.template_combo.currentIndexChanged.connect(self.on_template_selected)
        
        # Confidence display
        self.confidence_bar = QProgressBar()
        self.confidence_bar.setRange(0, 100)
        self.confidence_bar.setFormat("Confidence: %p%")

        # Strategy selection and params
        self.strategy_combo = QComboBox()
        self.strategy_combo.addItem("Template (classic)", "template")
        self.strategy_combo.addItem("Feature (scale/rotation)", "feature")
        self.min_inliers_spin = QSpinBox()
        self.min_inliers_spin.setRange(4, 1000)
        self.min_inliers_spin.setValue(12)
        self.ratio_spin = QDoubleSpinBox()
        self.ratio_spin.setRange(0.1, 0.99)
        self.ratio_spin.setDecimals(2)
        self.ratio_spin.setSingleStep(0.05)
        self.ratio_spin.setValue(0.75)
        self.ransac_spin = QDoubleSpinBox()
        self.ransac_spin.setRange(0.5, 20.0)
        self.ransac_spin.setDecimals(1)
        self.ransac_spin.setSingleStep(0.5)
        self.ransac_spin.setValue(4.0)
        
        # Preview area
        self.preview_label = QLabel()
        self.preview_label.setMinimumSize(320, 240)
        self.preview_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_label.setStyleSheet("background-color: #2d2d2d; border: 1px solid #3a3a3a;")
        
        # Controls
        controls_layout = QHBoxLayout()
        
        self.region_btn = QPushButton("Select Region")
        self.region_btn.clicked.connect(self.select_region)
        self.region_btn.setToolTip("Select a region of the screen to search in")
        
        self.start_btn = QPushButton("Start Preview")
        self.start_btn.clicked.connect(self.toggle_preview)
        self.start_btn.setEnabled(False)
        
        controls_layout.addWidget(self.region_btn)
        controls_layout.addWidget(self.start_btn)
        
        # Status
        self.region_status = QLabel("Region: Full Screen")
        self.region_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.status_label = QLabel("Select a template and click 'Start Preview'")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        # Add widgets to layout
        form_layout = QFormLayout()
        form_layout.addRow("Template:", self.template_combo)
        form_layout.addRow("Confidence:", self.confidence_bar)
        form_layout.addRow("Strategy:", self.strategy_combo)
        params_layout = QHBoxLayout()
        params_layout.addWidget(QLabel("Min Inliers"))
        params_layout.addWidget(self.min_inliers_spin)
        params_layout.addWidget(QLabel("Ratio"))
        params_layout.addWidget(self.ratio_spin)
        params_layout.addWidget(QLabel("RANSAC"))
        params_layout.addWidget(self.ransac_spin)
        form_layout.addRow("Params:", params_layout)

        # Visualizer controls (performance-friendly toggles)
        viz_layout = QHBoxLayout()
        self.keypoints_check = QCheckBox("Show Keypoints")
        self.keypoints_check.setChecked(False)
        viz_layout.addWidget(self.keypoints_check)
        viz_layout.addWidget(QLabel("Detector"))
        self.detector_combo = QComboBox()
        self.detector_combo.addItems(["ORB", "AKAZE", "SIFT"])
        viz_layout.addWidget(self.detector_combo)
        viz_layout.addStretch()
        form_layout.addRow("Visualizer:", viz_layout)

        # Capture backend selector (for performance)
        cap_layout = QHBoxLayout()
        cap_layout.addWidget(QLabel("Backend"))
        self.capture_backend_combo = QComboBox()
        self.capture_backend_combo.addItems(["Auto (best)", "QScreen", "MSS"])
        self.capture_backend_combo.setCurrentIndex(0)
        cap_layout.addWidget(self.capture_backend_combo)
        # Availability indicator
        self.backend_status = QLabel("Unknown")
        self.backend_status.setStyleSheet("color: #cfcfcf;")
        cap_layout.addWidget(self.backend_status)
        cap_layout.addStretch()
        form_layout.addRow("Capture:", cap_layout)
        # Update availability when backend changes
        try:
            self.capture_backend_combo.currentIndexChanged.connect(self.update_capture_backend_status)
        except Exception:
            pass

        # Metrics display (move overlay metrics into GUI)
        metrics_layout = QHBoxLayout()
        self.metric_inliers = QLabel("Inliers: -")
        self.metric_matches = QLabel("Matches: -")
        self.metric_conf = QLabel("Confidence: -")
        self.metric_reproj = QLabel("Reproj: -")
        for w in (self.metric_inliers, self.metric_matches, self.metric_conf, self.metric_reproj):
            w.setStyleSheet("color: #cfcfcf;")
        metrics_layout.addWidget(self.metric_inliers)
        metrics_layout.addWidget(self.metric_matches)
        metrics_layout.addWidget(self.metric_conf)
        metrics_layout.addWidget(self.metric_reproj)
        metrics_layout.addStretch()
        form_layout.addRow("Metrics:", metrics_layout)

        # Debug info panel
        debug_group = QGroupBox("Debug")
        debug_layout = QGridLayout()
        self.debug_target = QLabel("Target: -")
        self.debug_geom = QLabel("Geom: -")
        self.debug_local = QLabel("Local: -")
        self.debug_size = QLabel("Size: -")
        self.debug_dpr = QLabel("DPR: -")
        self.debug_pm = QLabel("Pixmap: -")
        self.debug_fallback = QLabel("Fallback: -")
        for i, w in enumerate([self.debug_target, self.debug_geom, self.debug_local, self.debug_size, self.debug_dpr, self.debug_pm, self.debug_fallback]):
            w.setStyleSheet("color: #cfcfcf;")
            row = i // 2; col = (i % 2) * 1
            debug_layout.addWidget(w, row, col)
        debug_group.setLayout(debug_layout)
        form_layout.addRow(debug_group)
        
        template_layout.addLayout(form_layout)
        template_layout.addWidget(self.preview_label)
        template_layout.addLayout(controls_layout)
        template_layout.addWidget(self.region_status)
        template_layout.addWidget(self.status_label)
        
        layout.addWidget(template_group)
        layout.addStretch()
        # Initial availability update
        try:
            self.update_capture_backend_status()
        except Exception:
            pass

        # (removed) Scheduler timer setup here was incorrect; initialized in MainWindow later

    def update_capture_backend_status(self):
        """Check whether the selected capture backend is available and update the UI label."""
        name = self.capture_backend_combo.currentText() if hasattr(self, 'capture_backend_combo') else "Auto (best)"
        available = False
        detail = ""
        try:
            if name == "MSS":
                try:
                    import mss
                    with mss.mss() as _sct:
                        pass
                    available = True
                except Exception as e:
                    detail = f"mss failed: {e}"
                    available = False
            elif name == "QScreen":
                # PyQt6 QScreen is always available
                available = True
            else:  # Auto (best)
                # Prefer MSS, else QScreen
                try:
                    import mss
                    with mss.mss() as _sct:
                        available = True
                        detail = "Using MSS"
                except Exception as e2:
                    available = True
                    detail = "Using QScreen"
        except Exception as e:
            available = False
            detail = f"error: {e}"

        self.backend_status.setText("Available" if available else "Unavailable")
        self.backend_status.setStyleSheet("color: #4caf50;" if available else "color: #ff6b6b;")
        if detail:
            self.backend_status.setToolTip(detail)
    
    def on_template_selected(self, index):
        template_path = self.template_combo.currentData()
        if template_path:  # Check if a valid template is selected (not the placeholder)
            self.template_path = template_path
            self.start_btn.setEnabled(True)
            self.status_label.setText("Click 'Start Preview' to begin")
        else:
            self.template_path = ""
            self.start_btn.setEnabled(False)
            self.status_label.setText("Select a template and click 'Start Preview'")
    
    def toggle_preview(self):
        if self.timer.isActive():
            self.timer.stop()
            self.start_btn.setText("Start Preview")
            self.status_label.setText("Preview stopped")
        else:
            if self.template_path and os.path.exists(self.template_path):
                self.timer.start(100)  # Update every 100ms
                self.start_btn.setText("Stop Preview")
                self.status_label.setText("Preview active - Press F8 to stop")
    
    def select_region(self):
        """Open a dialog to select a region of the screen."""
        dialog = ScreenCaptureDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            rect = dialog.get_capture_rect()
            if rect and rect.isValid():
                # Convert QRect to (x, y, width, height) tuple
                self.search_region = (rect.x(), rect.y(), rect.width(), rect.height())
                self.region_status.setText(f"Region: X={rect.x()}, Y={rect.y()}, W={rect.width()}, H={rect.height()}")
            else:
                self.search_region = None
                self.region_status.setText("Region: Full Screen")
    
    def update_preview(self):
        if not self.template_path or not os.path.exists(self.template_path):
            self.timer.stop()
            self.start_btn.setText("Start Preview")
            self.status_label.setText("Template file not found")
            return
        
        try:
            # Log the current tester state
            logger.info(f"TemplateTester: start preview, region={self.search_region}, strategy={self.strategy_combo.currentData()}")
            # Capture screen with multi-monitor support
            def capture_region_bgr(x: int, y: int, w: int, h: int) -> np.ndarray:
                backend = self.capture_backend_combo.currentText() if hasattr(self, 'capture_backend_combo') else "Auto (best)"
                # DXCAM removed
                # Try MSS if selected or Auto
                if backend in ("Auto (best)", "MSS"):
                    try:
                        import mss
                        with mss.mss() as sct:
                            bbox = {"left": int(x), "top": int(y), "width": int(w), "height": int(h)}
                            img = sct.grab(bbox)
                            arr = np.array(img)  # BGRA
                            self.debug_fallback.setText("MSS region")
                            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
                    except Exception:
                        pass
                # Fallback to QScreen DPI-aware region capture
                screens = QGuiApplication.screens()
                best = None
                best_area = -1
                for sc in screens:
                    g = sc.geometry()
                    gx1, gy1, gx2, gy2 = g.x(), g.y(), g.x() + g.width(), g.y() + g.height()
                    ix1, iy1 = max(x, gx1), max(y, gy1)
                    ix2, iy2 = min(x + w, gx2), min(y + h, gy2)
                    if ix2 > ix1 and iy2 > iy1:
                        area = (ix2 - ix1) * (iy2 - iy1)
                        if area > best_area:
                            best_area = area
                            best = sc
                target = best if best is not None else QGuiApplication.primaryScreen()
                geom = target.geometry()
                local_x = max(0, x - geom.x())
                local_y = max(0, y - geom.y())
                grab_w = max(1, min(w, geom.width() - local_x))
                grab_h = max(1, min(h, geom.height() - local_y))
                dpr = target.devicePixelRatio()
                pm = target.grabWindow(0, int(local_x * dpr), int(local_y * dpr), int(grab_w * dpr), int(grab_h * dpr))
                qi = pm.toImage().convertToFormat(QImage.Format.Format_BGR888)
                height = qi.height(); width = qi.width(); bpl = qi.bytesPerLine()
                size_bytes = qi.sizeInBytes() if hasattr(qi, 'sizeInBytes') else (bpl * height)
                bits = qi.bits(); bits.setsize(size_bytes)
                buf = np.frombuffer(bits, dtype=np.uint8)
                img_row = buf.reshape((height, bpl))
                img_row = img_row[:, :width * 3]
                arr = img_row.reshape((height, width, 3)).copy()
                try:
                    target_idx = next((i for i, sc in enumerate(QGuiApplication.screens(), start=1) if sc is target), None)
                except Exception:
                    target_idx = None
                self.debug_target.setText(f"Target: Screen {target_idx if target_idx else '-'}")
                self.debug_geom.setText(f"Geom: ({geom.x()},{geom.y()},{geom.width()}x{geom.height()})")
                self.debug_local.setText(f"Local: ({local_x},{local_y},{grab_w}x{grab_h})")
                self.debug_size.setText(f"Size: {width}x{height}, bpl={bpl}")
                self.debug_dpr.setText(f"DPR: {dpr:.2f}")
                self.debug_pm.setText(f"Pixmap: {'null' if pm.isNull() else 'ok'}")
                self.debug_fallback.setText("Fallback: QScreen")
                logger.info(f"TemplateTester: target_screen={target_idx}, geom=({geom.x()},{geom.y()},{geom.width()}x{geom.height()}), local=({local_x},{local_y},{grab_w}x{grab_h}), size={width}x{height}, dpr={dpr}, pm_null={pm.isNull()}")
                return arr

            if self.search_region:
                rx, ry, rw, rh = self.search_region
                screenshot = capture_region_bgr(rx, ry, rw, rh)
                if screenshot is None:
                    # Fallback capture for region using PIL/pyautogui
                    try:
                        img = ImageGrab.grab(bbox=(rx, ry, rx + rw, ry + rh))
                        self.debug_fallback.setText("Fallback: PIL bbox")
                        logger.info("TemplateTester: region fallback via PIL bbox")
                    except Exception:
                        img = pyautogui.screenshot(region=(rx, ry, rw, rh))
                        self.debug_fallback.setText("Fallback: pyautogui region")
                        logger.info("TemplateTester: region fallback via pyautogui region")
                    screenshot = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
            else:
                # Full virtual desktop snapshot (all monitors)
                # Try fast backends first
                backend = self.capture_backend_combo.currentText() if hasattr(self, 'capture_backend_combo') else "Auto (best)"
                # DXCAM removed
                if backend == "MSS":
                    try:
                        import mss
                        with mss.mss() as sct:
                            mon = sct.monitors[0]  # virtual desktop
                            img = sct.grab(mon)
                            arr = np.array(img)
                            screenshot = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
                            self.debug_fallback.setText("MSS full")
                    except Exception:
                        screenshot = None
                else:
                    screenshot = None
                if screenshot is None:
                    # Fallback via stitched QScreen captures
                    screens = QGuiApplication.screens()
                    if screens:
                        x0 = min(sc.geometry().x() for sc in screens)
                        y0 = min(sc.geometry().y() for sc in screens)
                        x1 = max(sc.geometry().x() + sc.geometry().width() for sc in screens)
                        y1 = max(sc.geometry().y() + sc.geometry().height() for sc in screens)
                        total_w, total_h = x1 - x0, y1 - y0
                        canvas = np.zeros((total_h, total_w, 3), dtype=np.uint8)
                        for sc in screens:
                            g = sc.geometry()
                            pm = sc.grabWindow(0)
                            if pm.isNull():
                                continue
                screens = QGuiApplication.screens()
                if screens:
                    x0 = min(sc.geometry().x() for sc in screens)
                    y0 = min(sc.geometry().y() for sc in screens)
                    x1 = max(sc.geometry().x() + sc.geometry().width() for sc in screens)
                    y1 = max(sc.geometry().y() + sc.geometry().height() for sc in screens)
                    total_w, total_h = x1 - x0, y1 - y0
                    canvas = np.zeros((total_h, total_w, 3), dtype=np.uint8)
                    for sc in screens:
                        g = sc.geometry()
                        pm = sc.grabWindow(0)
                        if pm.isNull():
                            continue
                        qi = pm.toImage().convertToFormat(QImage.Format.Format_BGR888)
                        h = qi.height(); w = qi.width(); bpl = qi.bytesPerLine()
                        size_bytes = getattr(qi, 'sizeInBytes', None)
                        size_bytes = size_bytes() if callable(size_bytes) else (bpl * h)
                        bits = qi.bits(); bits.setsize(size_bytes)
                        buf = np.frombuffer(bits, dtype=np.uint8)
                        img_row = buf.reshape((h, bpl))
                        img_row = img_row[:, :w * 3]
                        arr = img_row.reshape((h, w, 3))
                        off_x = g.x() - x0
                        off_y = g.y() - y0
                        y_end = min(off_y + h, total_h)
                        x_end = min(off_x + w, total_w)
                        canvas[off_y:y_end, off_x:x_end] = arr[:y_end - off_y, :x_end - off_x]
                    screenshot = canvas
                    self.debug_fallback.setText("Fallback: stitched full desktop")
                    logger.info("TemplateTester: full desktop via stitched QScreen captures")
                else:
                    img = pyautogui.screenshot()
                    screenshot = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
                    self.debug_fallback.setText("Fallback: pyautogui full")
                    logger.info("TemplateTester: full desktop via pyautogui screenshot")
            if screenshot is None or screenshot.size == 0:
                self.status_label.setText("Failed to capture screen")
                logger.warning("TemplateTester: empty screenshot frame")
                return
                
            # Load template
            template = cv2.imread(self.template_path, cv2.IMREAD_COLOR)
            if template is None:
                raise ValueError("Failed to load template image")
            
            display_img = screenshot.copy()
            strategy = self.strategy_combo.currentData()
            if strategy == "feature":
                # Feature-based preview: draw inlier matches and polygon
                try:
                    # Choose detector based on UI (ORB or AKAZE)
                    use_alg = self.detector_combo.currentText() if hasattr(self, 'detector_combo') else "ORB"
                    if use_alg == "AKAZE":
                        detector = cv2.AKAZE_create()
                        norm = cv2.NORM_HAMMING
                    elif use_alg == "SIFT":
                        detector = cv2.SIFT_create()
                        norm = cv2.NORM_L2
                    else:
                        detector = cv2.ORB_create(nfeatures=1500)
                        norm = cv2.NORM_HAMMING
                    gray_screen = cv2.cvtColor(screenshot, cv2.COLOR_BGR2GRAY)
                    gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
                    kps1, des1 = detector.detectAndCompute(gray_template, None)
                    kps2, des2 = detector.detectAndCompute(gray_screen, None)
                    # Draw scene keypoints on the preview for visual debugging (optional)
                    if getattr(self, 'keypoints_check', None) and self.keypoints_check.isChecked():
                        try:
                            keypoint_img = cv2.drawKeypoints(display_img, kps2 if kps2 is not None else [], None,
                                                             color=(255, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)
                            if keypoint_img is not None:
                                display_img = keypoint_img
                        except Exception:
                            pass
                    bf = cv2.BFMatcher(norm, crossCheck=False)
                    matches = bf.knnMatch(des1, des2, k=2) if des1 is not None and des2 is not None else []
                    good = []
                    ratio_thresh = float(self.ratio_spin.value())
                    for m_n in matches:
                        if len(m_n) == 2:
                            m, n = m_n
                            if m.distance < ratio_thresh * n.distance:
                                good.append(m)
                    inliers = 0
                    polygon = None
                    mean_err = None
                    median_err = None
                    if len(good) >= int(self.min_inliers_spin.value()):
                        src_pts = np.float32([kps1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                        dst_pts = np.float32([kps2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, float(self.ransac_spin.value()))
                        if H is not None and mask is not None:
                            inliers = int(np.sum(mask))
                            h_tpl, w_tpl = gray_template.shape[:2]
                            pts = np.float32([[0, 0], [w_tpl, 0], [w_tpl, h_tpl], [0, h_tpl]]).reshape(-1, 1, 2)
                            dst = cv2.perspectiveTransform(pts, H)
                            polygon = dst.reshape(-1, 2).astype(int)
                            # Draw polygon
                            for i in range(4):
                                pt1 = tuple(polygon[i])
                                pt2 = tuple(polygon[(i + 1) % 4])
                                cv2.line(display_img, pt1, pt2, (0, 255, 0), 2)
                            # Draw inlier points
                            for j, m in enumerate(good):
                                if mask[j]:
                                    p = tuple(np.int32(kps2[m.trainIdx].pt))
                                    cv2.circle(display_img, p, 3, (0, 0, 255), -1)
                            # Compute reprojection error on inliers
                            try:
                                inlier_mask = mask.ravel().astype(bool)
                                src_in = src_pts[inlier_mask]
                                dst_in = dst_pts[inlier_mask]
                                proj_in = cv2.perspectiveTransform(src_in, H)
                                diffs = proj_in - dst_in
                                dists = np.linalg.norm(diffs, axis=2)  # (N,1)
                                if dists.size > 0:
                                    mean_err = float(np.mean(dists))
                                    median_err = float(np.median(dists))
                            except Exception:
                                pass
                    # Confidence as inliers/good
                    conf_pct = 0
                    if len(good) > 0:
                        conf_pct = int((inliers / len(good)) * 100)
                    self.confidence_bar.setValue(conf_pct)
                    # Update GUI metrics (so they remain visible even in small regions)
                    self.metric_inliers.setText(f"Inliers: {inliers}")
                    self.metric_matches.setText(f"Matches: {len(good)}")
                    self.metric_conf.setText(f"Confidence: {conf_pct}%")
                    self.metric_reproj.setText(f"Reproj: {median_err:.2f}px" if median_err is not None else "Reproj: N/A")
                    self.status_label.setText("Feature preview active")
                except Exception as e:
                    self.status_label.setText(f"Feature preview error: {e}")
                    self.confidence_bar.setValue(0)
                    self.metric_inliers.setText("Inliers: -")
                    self.metric_matches.setText("Matches: -")
                    self.metric_conf.setText("Confidence: -")
                    self.metric_reproj.setText("Reproj: -")
            else:
                # Classic template matching preview
                gray_screen = cv2.cvtColor(screenshot, cv2.COLOR_BGR2GRAY)
                gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
                result = cv2.matchTemplate(gray_screen, gray_template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                confidence = max_val * 100
                self.confidence_bar.setValue(int(confidence))
                h, w = template.shape[:2]
                top_left = max_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(display_img, top_left, bottom_right, (0, 255, 0), 2)
                text = f"{confidence:.1f}%"
                cv2.putText(display_img, text, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # Update GUI metrics for classic strategy
                self.metric_inliers.setText("Inliers: N/A")
                self.metric_matches.setText("Matches: N/A")
                self.metric_conf.setText(f"Confidence: {int(confidence)}%")
                self.metric_reproj.setText("Reproj: N/A")
                self.status_label.setText("Classic preview active")
            
            # Draw search region border if a region is selected (display_img is already cropped to region)
            if self.search_region:
                cv2.rectangle(display_img,
                              (0, 0),
                              (display_img.shape[1]-1, display_img.shape[0]-1),
                              (255, 0, 0), 1, cv2.LINE_AA)
            
            # For classic strategy, rectangle and text are drawn inside the classic branch above.
            
            # Convert to QImage and display
            height, width, channel = display_img.shape
            bytes_per_line = 3 * width
            q_img = QImage(display_img.data, width, height, bytes_per_line, QImage.Format.Format_BGR888)
            self.preview_label.setPixmap(QPixmap.fromImage(q_img).scaled(
                self.preview_label.width(), 
                self.preview_label.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            ))
            
        except Exception as e:
            self.status_label.setText(f"Error: {str(e)}")
            self.timer.stop()
            self.start_btn.setText("Start Preview")
    
    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)


class FailsafeSequenceEditor(QWidget):
    """Widget to edit the failsafe sequence."""
    def __init__(self, failsafe_data: dict = None, templates: List[str] = None, parent=None, groups: Optional[list] = None):
        super().__init__(parent)
        self.failsafe_data = failsafe_data or {"steps": []}
        self.templates = templates or []
        self.groups = groups or []
        self.step_widgets = []
        self.init_ui()
    
    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Header with add step button
        header_layout = QHBoxLayout()
        header_layout.addWidget(QLabel("Failsafe Sequence Steps"))
        header_layout.addStretch()
        
        add_step_btn = QPushButton("Add Step")
        add_step_btn.clicked.connect(self.add_step)
        header_layout.addWidget(add_step_btn)
        # Add Group Call button
        add_group_btn = QPushButton("Add Group Call")
        add_group_btn.clicked.connect(lambda: self.add_step({"call_group": ""}))
        header_layout.addWidget(add_group_btn)
        
        main_layout.addLayout(header_layout)
        
        # Scroll area for steps
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        
        self.steps_container = QWidget()
        self.steps_layout = QVBoxLayout(self.steps_container)
        self.steps_layout.setSpacing(10)
        self.steps_layout.setContentsMargins(5, 5, 5, 5)
        self.steps_layout.addStretch()
        
        self.scroll_area.setWidget(self.steps_container)
        main_layout.addWidget(self.scroll_area)
        
        # Load existing steps
        for step_data in self.failsafe_data.get("steps", []):
            self.add_step_from_data(step_data)
    
    def add_step(self, step_data: Optional[dict] = None):
        """Add a new step to the failsafe sequence.
        If step_data has {call_group: ...}, inserts a group-call step.
        """
        if step_data is None or not isinstance(step_data, dict):
            step_data = {"find": "", "required": True, "timeout": 10, "confidence": 0.9, "detection_strategy": "default", "monitor": None, "actions": [{"type": "click"}]}
        # Inject global search_region if available
        try:
            p = self.parent()
            while p is not None and p.__class__.__name__ != 'MainWindow':
                p = p.parent()
            if p is not None and hasattr(p, 'search_region') and isinstance(p.search_region, (list, tuple)) and len(p.search_region) == 4:
                step_data.setdefault('search_region', list(p.search_region))
            elif p is not None and isinstance(getattr(p, 'config', {}), dict):
                sr = (p.config or {}).get('search_region')
                if isinstance(sr, list) and len(sr) == 4:
                    step_data.setdefault('search_region', sr)
        except Exception:
            pass
        self.add_step_from_data(step_data)
    
    def add_step_from_data(self, step_data: dict):
        """Add a step from existing data."""
        # Route group-call steps to dedicated editor
        if isinstance(step_data, dict) and 'call_group' in step_data:
            step_widget = GroupCallStepEditor(step_data, self.groups, self)
        else:
            step_widget = StepEditor(step_data, self.templates, self)
        step_widget.setParent(self.steps_container)

        # Explicitly wire the delete button to the parent remover with this instance
        if hasattr(step_widget, 'remove_btn'):
            try:
                step_widget.remove_btn.clicked.disconnect()
            except Exception:
                pass
            step_widget.remove_btn.clicked.connect(lambda checked=False, sw=step_widget: self.remove_step(sw))
        
        # Insert before the stretch
        self.steps_layout.insertWidget(self.steps_layout.count() - 1, step_widget)
        self.step_widgets.append(step_widget)

        # Update numbering and layout
        self.update_step_numbers()
        self.steps_container.adjustSize()
        self.steps_container.update()
        self.scroll_area.update()
        # Persist after adding a step
        try:
            p = self.parent()
            while p is not None and not isinstance(p, MainWindow):
                p = p.parent()
            if isinstance(p, MainWindow) and not getattr(p, '_suppress_auto_save', False):
                p.update_config_from_ui()
                p.save_config()
        except Exception:
            pass

    def remove_step(self, step_editor):
        """Remove a step from the failsafe sequence.
        
        Args:
            step_editor: The StepEditor widget to remove
        """
        if step_editor in self.step_widgets:
            try:
                idx = self.step_widgets.index(step_editor)
                logger.info(f"FailsafeSequenceEditor: removing step at index {idx}")
            except Exception:
                logger.info("FailsafeSequenceEditor: removing step (index unknown)")
            # Remove from layout
            self.steps_layout.removeWidget(step_editor)
            
            # Remove from our list
            self.step_widgets.remove(step_editor)
            
            # Delete the widget
            step_editor.setParent(None)
            step_editor.deleteLater()
            
            # Update step numbers
            self.update_step_numbers()
            
            # Update the container size
            self.steps_container.adjustSize()
            self.steps_container.update()
            self.scroll_area.update()
            # Persist after removing a step
            try:
                p = self.parent()
                while p is not None and not isinstance(p, MainWindow):
                    p = p.parent()
                if isinstance(p, MainWindow) and not getattr(p, '_suppress_auto_save', False):
                    p.update_config_from_ui()
                    p.save_config()
            except Exception:
                pass
        else:
            # Fallback: if the widget isn't tracked, remove it anyway
            logger.info("FailsafeSequenceEditor: step not tracked; removing widget directly")
            try:
                self.steps_layout.removeWidget(step_editor)
            except Exception:
                pass
            try:
                step_editor.setParent(None)
                step_editor.deleteLater()
            except Exception:
                pass
            self.update_step_numbers()
            self.steps_container.adjustSize()
            self.steps_container.update()
            self.scroll_area.update()

    def update_step_numbers(self):
        """Update the step numbers in the failsafe UI."""
        for i, step in enumerate(self.step_widgets, 1):
            if hasattr(step, 'step_label'):
                step.step_label.setText(f"Step {i}")
    
    def get_failsafe_data(self) -> dict:
        """Get the current failsafe sequence data."""
        steps = []
        for step_widget in self.step_widgets:
            steps.append(step_widget.get_step_data())
        return {"steps": steps}

    def load_sequence(self, steps_list: list):
        """Replace the current steps with the provided list and refresh UI."""
        try:
            # Remove existing step widgets
            for sw in list(self.step_widgets):
                try:
                    self.remove_step(sw)
                except Exception:
                    try:
                        self.steps_layout.removeWidget(sw)
                        sw.setParent(None)
                        sw.deleteLater()
                    except Exception:
                        pass
            self.step_widgets.clear()
            # Add new steps
            for step_data in (steps_list or []):
                if isinstance(step_data, dict):
                    self.add_step_from_data(step_data)
            self.update_step_numbers()
            self.steps_container.adjustSize()
            self.steps_container.update()
            self.scroll_area.update()
        except Exception:
            pass

    def update_templates(self, templates: List[str]):
        """Update available templates list and refresh all step editors."""
        try:
            self.templates = templates or []
            for sw in self.step_widgets:
                if isinstance(sw, StepEditor):
                    sw.set_templates(self.templates)
        except Exception:
            pass


class MainWindow(QMainWindow):
    """Main application window."""
    def __init__(self):
        super().__init__()
        self.config = {}
        # Store the script/executable directory for relative paths (works for PyInstaller onedir)
        try:
            if getattr(sys, 'frozen', False):
                # Running from a bundled executable
                self.script_dir = os.path.dirname(sys.executable)
                # Ensure process working directory is the executable folder for consistent relative paths
                try:
                    os.chdir(self.script_dir)
                except Exception:
                    pass
            else:
                # Running from source
                self.script_dir = os.path.dirname(os.path.abspath(__file__))
        except Exception:
            # Fallback to current working directory
            self.script_dir = os.getcwd()
        # Default config path relative to script directory
        self.config_path = os.path.join(self.script_dir, "config.json")
        self.bot = ImageDetectionBot()
        self.search_region = None  # Will store the search region (x, y, width, height)
        self.worker = None  # Will store the worker thread
        self.f8_pressed = False  # Track F8 key state
        
        # Initialize directories relative to script location
        self.images_dir = os.path.join(self.script_dir, "images")
        self.failsafe_images_dir = os.path.join(self.script_dir, "failsafe_images")
        
        # Create necessary directories if they don't exist
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.failsafe_images_dir, exist_ok=True)
        
        self.init_ui()
        try:
            # Defer config load until UI is fully constructed
            QTimer.singleShot(0, lambda: self.load_config())
        except Exception:
            pass
        
        # Set up F8 key monitoring
        self.f8_timer = QTimer(self)
        self.f8_timer.timeout.connect(self.check_f8_key)
        self.f8_timer.start(100)  # Check every 100ms
    
    def init_ui(self):
        """Initialize the UI."""
        self.setWindowTitle("Image Detection Bot")
        self.setGeometry(100, 100, 1000, 700)  # Slightly smaller default size
        
        # Create a global shortcut for F8 to stop sequences
        self.f8_shortcut = QShortcut(QKeySequence("F8"), self)
        self.f8_shortcut.activated.connect(self.stop_sequence)
        self.f8_shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)
        
        # Create main widget and layout
        main_widget = QWidget()
        main_layout = QVBoxLayout(main_widget)
        # Set central widget early to ensure window shows even if init bails early
        try:
            self.setCentralWidget(main_widget)
        except Exception:
            pass
        
        # Toolbar
        toolbar = QToolBar()
        self.addToolBar(toolbar)

        # Quick config actions on toolbar
        cfg_new_action = QAction("New", self)
        cfg_new_action.setToolTip("Create a new configuration")
        cfg_new_action.triggered.connect(self.new_config)
        cfg_open_action = QAction("Open", self)
        cfg_open_action.setToolTip("Open configuration file")
        cfg_open_action.triggered.connect(self.open_config)
        cfg_save_action = QAction("Save", self)
        cfg_save_action.setToolTip("Save current configuration")
        cfg_save_action.triggered.connect(self.save_config)
        cfg_save_as_action = QAction("Save As", self)
        cfg_save_as_action.setToolTip("Save configuration to a new file")
        cfg_save_as_action.triggered.connect(self.save_config_as)
        toolbar.addAction(cfg_new_action)
        toolbar.addAction(cfg_open_action)
        toolbar.addAction(cfg_save_action)
        toolbar.addAction(cfg_save_as_action)
        toolbar.addSeparator()
        
        # Region selection action
        self.region_action = QAction("Select Region", self)
        self.region_action.setCheckable(True)
        self.region_action.toggled.connect(self.toggle_region_selection)
        toolbar.addAction(self.region_action)
        
        # Add separator
        toolbar.addSeparator()
        # Monitor selection
        monitor_label = QLabel("Monitor:")
        toolbar.addWidget(monitor_label)
        self.monitor_combo = QComboBox()
        # Track monitor regions for "All Monitors" bounds display
        self.monitor_regions = []
        # Options: All Monitors, None (full screen), then individual screens
        self.monitor_combo.addItem("All Monitors", "ALL")
        self.monitor_combo.addItem("None (Full Screen)", None)
        try:
            screens = QGuiApplication.screens()
            for idx, screen in enumerate(screens, start=1):
                geom = screen.geometry()
                item_label = f"Screen {idx} ({geom.x()},{geom.y()},{geom.width()}x{geom.height()})"
                region_tuple = (geom.x(), geom.y(), geom.width(), geom.height())
                self.monitor_combo.addItem(item_label, region_tuple)
                self.monitor_regions.append(region_tuple)
        except Exception as e:
            logger.debug(f"Failed to list screens: {e}")
        self.monitor_combo.currentIndexChanged.connect(self.on_monitor_selected)
        toolbar.addWidget(self.monitor_combo)

        # Monitor info action
        self.monitor_info_action = QAction("Monitor Info", self)
        self.monitor_info_action.triggered.connect(self.show_monitor_info)
        toolbar.addAction(self.monitor_info_action)
        
        # Stop action
        self.stop_action = QAction("Stop", self)
        self.stop_action.setEnabled(False)
        self.stop_action.triggered.connect(self.stop_sequence)
        toolbar.addAction(self.stop_action)
        
        # Status bar widgets
        status_bar = self.statusBar()
        
        # Region status label
        self.region_status = QLabel("Region: Full Screen")
        status_bar.addPermanentWidget(self.region_status)
        
        # Mouse position label
        self.mouse_pos_label = QLabel("X: 0, Y: 0")
        status_bar.addPermanentWidget(self.mouse_pos_label)

        # Next scheduled run indicator
        self.scheduler_label = QLabel("Next: (none)")
        status_bar.addPermanentWidget(self.scheduler_label)
        
        # Set up a timer to update mouse position
        self.mouse_timer = QTimer(self)
        self.mouse_timer.timeout.connect(self.update_mouse_position)
        self.mouse_timer.start(100)  # Update every 100ms
        
        # Set up F8 global shortcut for stopping sequences
        self.f8_shortcut = QShortcut(QKeySequence("F8"), self)
        self.f8_shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)  # Make it work globally
        self.f8_shortcut.activated.connect(self.stop_sequence)
        
        # Also handle F8 via key press event as a fallback
        self.f8_pressed = False
        
        # Main content area
        content_widget = QWidget()
        content_layout = QVBoxLayout(content_widget)
        content_layout.setContentsMargins(0, 0, 0, 0)
        
        # QMainWindow manages toolbars; no need to add to central layout
        main_layout.addWidget(content_widget)
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        content_layout.addWidget(self.tab_widget)
        
        # Create tabs
        self.sequences_tab = QWidget()
        self.groups_tab = QWidget()
        self.failsafe_tab = QWidget()
        self.templates_tab = QWidget()
        
        # Set up sequences tab
        sequences_tab_layout = QHBoxLayout(self.sequences_tab)
        
        # Left sidebar for sequences
        sidebar = QWidget()
        sidebar.setFixedWidth(250)
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        
        # Sequences section
        sequences_group = QGroupBox("Sequences")
        sequences_layout = QVBoxLayout()
        
        self.sequences_list = QListWidget()
        self.sequences_list.currentItemChanged.connect(self.on_sequence_selected)
        
        # Button layout for sequence operations
        sequence_btns_layout = QHBoxLayout()
        
        self.add_sequence_btn = QPushButton("Add")
        self.add_sequence_btn.clicked.connect(self.add_sequence)
        
        self.duplicate_sequence_btn = QPushButton("Duplicate")
        self.duplicate_sequence_btn.setEnabled(False)
        self.duplicate_sequence_btn.clicked.connect(self.duplicate_sequence)
        
        self.rename_sequence_btn = QPushButton("Rename")
        self.rename_sequence_btn.setEnabled(False)
        self.rename_sequence_btn.clicked.connect(self.rename_sequence)
        
        self.remove_sequence_btn = QPushButton("Del")
        self.remove_sequence_btn.setEnabled(False)
        self.remove_sequence_btn.clicked.connect(self.remove_sequence)
        
        sequence_btns_layout.addWidget(self.add_sequence_btn)
        sequence_btns_layout.addWidget(self.duplicate_sequence_btn)
        sequence_btns_layout.addWidget(self.rename_sequence_btn)
        sequence_btns_layout.addWidget(self.remove_sequence_btn)
        
        # Execution control
        exec_control = QVBoxLayout()
        
        # Non-required steps behavior
        self.non_required_wait_checkbox = QCheckBox("Wait full timeout for non-required steps")
        self.non_required_wait_checkbox.setToolTip(
            "When enabled, non-required steps will wait the full timeout period.\n"
            "When disabled, non-required steps will skip immediately if template is not found."
        )
        self.non_required_wait_checkbox.setChecked(False)  # Default to skip immediately

        # Branch debug traces toggle (runtime)
        self.branch_debug_checkbox = QCheckBox("Branch Debug Traces")
        self.branch_debug_checkbox.setToolTip(
            "When enabled, logs which conditional branch runs (ELSE vs IF_NOT) per action.\n"
            "Runtime toggle; no restart required. Also saved into config when you save."
        )
        self.branch_debug_checkbox.setChecked(False)
        try:
            self.branch_debug_checkbox.stateChanged.connect(self.on_branch_debug_toggled)
        except Exception:
            pass
        
        # Loop control
        loop_control = QHBoxLayout()
        self.loop_checkbox = QCheckBox("Loop")
        self.loop_count_spin = QSpinBox()
        self.loop_count_spin.setMinimum(1)
        self.loop_count_spin.setMaximum(999999)  # Increased from 9999 to 999999
        self.loop_count_spin.setValue(1)
        self.loop_count_spin.setEnabled(False)
        self.loop_checkbox.stateChanged.connect(
            lambda state: self.loop_count_spin.setEnabled(state == Qt.CheckState.Checked.value)
        )
        loop_control.addWidget(self.loop_checkbox)
        loop_control.addWidget(QLabel("Iterations:"))
        loop_control.addWidget(self.loop_count_spin)
        loop_control.addStretch()
        
        # Add widgets to layout
        exec_control.addWidget(self.non_required_wait_checkbox)
        exec_control.addWidget(self.branch_debug_checkbox)
        exec_control.addLayout(loop_control)
        
        # Run button
        run_sequence_btn = QPushButton("Run")
        run_sequence_btn.clicked.connect(self.run_sequence)
        
        # Add widgets to sequences layout
        sequences_layout.addWidget(self.sequences_list)
        sequences_layout.addLayout(sequence_btns_layout)
        sequences_layout.addLayout(exec_control)  # Changed from loop_control_layout to exec_control
        sequences_layout.addWidget(run_sequence_btn)
        sequences_group.setLayout(sequences_layout)
        
        # Add sequences group to sidebar
        sidebar_layout.addWidget(sequences_group)
        
        # Add sidebar to sequences tab
        sequences_tab_layout.addWidget(sidebar)
        
        # Add sequence editor stack to sequences tab
        self.sequence_stack = QStackedWidget()
        sequences_tab_layout.addWidget(self.sequence_stack)

        # Set up groups tab
        groups_tab_layout = QHBoxLayout(self.groups_tab)

        # Left sidebar for groups
        groups_sidebar = QWidget()
        groups_sidebar.setFixedWidth(250)
        groups_sidebar_layout = QVBoxLayout(groups_sidebar)
        groups_sidebar_layout.setContentsMargins(0, 0, 0, 0)

        # Groups section
        groups_group = QGroupBox("Groups")
        groups_layout = QVBoxLayout()

        self.groups_list = QListWidget()
        self.groups_list.currentItemChanged.connect(self.on_group_selected)

        # Button layout for group operations
        groups_btns_layout = QHBoxLayout()

        self.add_group_btn = QPushButton("Add")
        self.add_group_btn.clicked.connect(self.add_group)

        self.rename_group_btn = QPushButton("Rename")
        self.rename_group_btn.setEnabled(False)
        self.rename_group_btn.clicked.connect(self.rename_group)

        self.remove_group_btn = QPushButton("Del")
        self.remove_group_btn.setEnabled(False)
        self.remove_group_btn.clicked.connect(self.remove_group)

        groups_btns_layout.addWidget(self.add_group_btn)
        groups_btns_layout.addWidget(self.rename_group_btn)
        groups_btns_layout.addWidget(self.remove_group_btn)

        groups_layout.addWidget(self.groups_list)
        groups_layout.addLayout(groups_btns_layout)
        groups_group.setLayout(groups_layout)

        groups_sidebar_layout.addWidget(groups_group)
        groups_tab_layout.addWidget(groups_sidebar)

        # Add group editor stack to groups tab
        self.group_stack = QStackedWidget()
        groups_tab_layout.addWidget(self.group_stack)
        
        # Set up failsafe tab
        failsafe_tab_layout = QHBoxLayout(self.failsafe_tab)
        
        # Left sidebar for failsafe configuration
        failsafe_sidebar = QWidget()
        failsafe_sidebar.setFixedWidth(250)
        failsafe_sidebar_layout = QVBoxLayout(failsafe_sidebar)
        failsafe_sidebar_layout.setContentsMargins(0, 0, 0, 0)
        
        # Failsafe configuration section
        failsafe_config_group = QGroupBox("Failsafe Configuration")
        failsafe_config_layout = QVBoxLayout()
        
        # Enable failsafe checkbox
        self.failsafe_enable_checkbox = QCheckBox("Enable Failsafe")
        self.failsafe_enable_checkbox.setChecked(False)
        failsafe_config_layout.addWidget(self.failsafe_enable_checkbox)
        # Wire checkbox to toggle UI (use toggled(bool) only to avoid duplicate/oscillating calls)
        try:
            self.failsafe_enable_checkbox.toggled.connect(self.toggle_failsafe_ui)
        except Exception:
            pass
        
        # Failsafe template selection
        fs_template_layout = QHBoxLayout()
        fs_template_layout.addWidget(QLabel("Template:"))
        self.failsafe_template_combo = QComboBox()
        self.failsafe_template_combo.setEnabled(False)
        # Update preview when selection changes
        self.failsafe_template_combo.currentTextChanged.connect(self.update_failsafe_preview)
        fs_template_layout.addWidget(self.failsafe_template_combo)
        failsafe_config_layout.addLayout(fs_template_layout)

        # Failsafe template name and image controls
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Name:"))
        self.failsafe_template_name_edit = QLineEdit()
        self.failsafe_template_name_edit.setPlaceholderText("Enter template name")
        self.failsafe_template_name_edit.setEnabled(False)
        name_layout.addWidget(self.failsafe_template_name_edit)
        failsafe_config_layout.addLayout(name_layout)

        image_btns_layout = QHBoxLayout()
        self.fs_capture_btn = QPushButton("Capture")
        self.fs_capture_btn.setEnabled(False)
        self.fs_capture_btn.clicked.connect(self.capture_failsafe_image)
        self.fs_select_btn = QPushButton("Load Image")
        self.fs_select_btn.setEnabled(False)
        self.fs_select_btn.clicked.connect(self.select_failsafe_image)
        image_btns_layout.addWidget(self.fs_capture_btn)
        image_btns_layout.addWidget(self.fs_select_btn)
        image_btns_layout.addStretch()
        failsafe_config_layout.addLayout(image_btns_layout)

        # Preview for selected failsafe template
        self.failsafe_preview = QLabel("No preview")
        self.failsafe_preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.failsafe_preview.setMinimumHeight(120)
        self.failsafe_preview.setStyleSheet("border: 1px solid #3a3a3a; border-radius: 3px; padding: 6px;")
        failsafe_config_layout.addWidget(self.failsafe_preview)

        # Confidence level
        fs_conf_layout = QHBoxLayout()
        fs_conf_layout.addWidget(QLabel("Confidence:"))
        self.failsafe_conf_spin = QDoubleSpinBox()
        self.failsafe_conf_spin.setRange(0.1, 1.0)
        self.failsafe_conf_spin.setSingleStep(0.05)
        self.failsafe_conf_spin.setValue(0.9)
        self.failsafe_conf_spin.setDecimals(2)
        self.failsafe_conf_spin.setEnabled(False)
        fs_conf_layout.addWidget(self.failsafe_conf_spin)
        fs_conf_layout.addStretch()
        failsafe_config_layout.addLayout(fs_conf_layout)
        
        # Region selection
        fs_region_layout = QHBoxLayout()
        self.failsafe_region_btn = QPushButton("Select Region")
        self.failsafe_region_btn.setEnabled(False)
        self.failsafe_region_label = QLabel("Region: Full Screen")
        fs_region_layout.addWidget(self.failsafe_region_btn)
        fs_region_layout.addWidget(self.failsafe_region_label)
        # Show region preview button
        self.failsafe_show_region_btn = QPushButton("Show Region")
        self.failsafe_show_region_btn.setToolTip("Preview the current failsafe search region on screen")
        self.failsafe_show_region_btn.setEnabled(False)
        fs_region_layout.addWidget(self.failsafe_show_region_btn)
        # Wire region selection
        try:
            self.failsafe_region_btn.clicked.connect(self.select_failsafe_region)
            self.failsafe_show_region_btn.clicked.connect(self.show_failsafe_region_overlay)
        except Exception:
            pass
        fs_region_layout.addStretch()
        failsafe_config_layout.addLayout(fs_region_layout)
        
        failsafe_config_group.setLayout(failsafe_config_layout)
        failsafe_sidebar_layout.addWidget(failsafe_config_group)
        
        # Failsafe sequence control
        fs_control_group = QGroupBox("Failsafe Sequence Control")
        fs_control_layout = QVBoxLayout()
        
        # Test failsafe button
        self.test_failsafe_btn = QPushButton("Test Failsafe")
        self.test_failsafe_btn.setEnabled(False)
        fs_control_layout.addWidget(self.test_failsafe_btn)
        # Wire test button
        try:
            self.test_failsafe_btn.clicked.connect(self.test_failsafe_sequence)
        except Exception:
            pass
        
        fs_control_group.setLayout(fs_control_layout)
        failsafe_sidebar_layout.addWidget(fs_control_group)
        
        failsafe_sidebar_layout.addStretch()
        
        # Add sidebar to failsafe tab
        failsafe_tab_layout.addWidget(failsafe_sidebar)
        
        # Add failsafe sequence editor stack
        self.failsafe_sequence_stack = QStackedWidget()
        # Add a welcome/placeholder page at index 0
        placeholder = QWidget()
        ph_layout = QVBoxLayout(placeholder)
        ph_layout.addWidget(QLabel("Enable Failsafe to edit the sequence"))
        ph_layout.addStretch()
        self.failsafe_sequence_stack.addWidget(placeholder)
        failsafe_tab_layout.addWidget(self.failsafe_sequence_stack)
        # Sync UI with current checkbox state
        try:
            # Avoid any persistence during initial UI sync
            try:
                self._suppress_auto_save = True
            except Exception:
                pass
            self.toggle_failsafe_ui(self.failsafe_enable_checkbox.isChecked())
            try:
                self._suppress_auto_save = False
            except Exception:
                pass
        except Exception:
            pass
        
        # Set up templates tab
        templates_tab_layout = QHBoxLayout(self.templates_tab)
        
        # Left sidebar for templates
        templates_sidebar = QWidget()
        templates_sidebar.setFixedWidth(250)
        templates_sidebar_layout = QVBoxLayout(templates_sidebar)
        templates_sidebar_layout.setContentsMargins(0, 0, 0, 0)
        
        # Templates section
        templates_group = QGroupBox("Templates")
        templates_layout = QVBoxLayout()
        
        self.templates_list = QListWidget()
        self.templates_list.currentItemChanged.connect(self.on_template_selected)
        
        # Button layout for template operations
        template_btns_layout = QHBoxLayout()
        
        add_template_btn = QPushButton("Add")
        add_template_btn.clicked.connect(self.add_template)
        
        self.delete_template_btn = QPushButton("Delete")
        self.delete_template_btn.clicked.connect(self.delete_template)
        self.delete_template_btn.setEnabled(False)
        
        template_btns_layout.addWidget(add_template_btn)
        template_btns_layout.addWidget(self.delete_template_btn)
        template_btns_layout.addStretch()
        
        templates_layout.addWidget(self.templates_list)
        templates_layout.addLayout(template_btns_layout)
        templates_group.setLayout(templates_layout)
        
        # Add templates group to sidebar
        templates_sidebar_layout.addWidget(templates_group)
        
        # Add template editor area
        self.template_stack = QStackedWidget()
        
        # Add widgets to templates tab
        templates_tab_layout.addWidget(templates_sidebar)
        templates_tab_layout.addWidget(self.template_stack)
        
        # Create template tester tab
        self.template_tester_tab = QWidget()
        self.template_tester = TemplateTester(self.bot, self.config.get('templates', {}).copy(), parent=self)
        tester_layout = QVBoxLayout(self.template_tester_tab)
        tester_layout.addWidget(self.template_tester)
        # Ensure the template list is populated
        self.update_template_tester_templates()

        # Create break settings tab
        self.break_tab = QWidget()
        break_layout = QVBoxLayout(self.break_tab)
        break_group = QGroupBox("Break Settings")
        break_group_layout = QVBoxLayout()

        # Enable checkbox
        self.break_enabled_checkbox = QCheckBox("Enable Max Runtime")
        self.break_enabled_checkbox.setChecked(False)
        break_group_layout.addWidget(self.break_enabled_checkbox)

        # Hours/minutes inputs
        time_layout = QHBoxLayout()
        time_layout.addWidget(QLabel("Max runtime:"))
        self.break_hours_spin = QSpinBox()
        self.break_hours_spin.setRange(0, 999)
        self.break_hours_spin.setValue(0)
        time_layout.addWidget(self.break_hours_spin)
        time_layout.addWidget(QLabel("hours"))
        self.break_minutes_spin = QSpinBox()
        self.break_minutes_spin.setRange(0, 59)
        self.break_minutes_spin.setValue(0)
        time_layout.addWidget(self.break_minutes_spin)
        time_layout.addWidget(QLabel("minutes"))
        # Seconds input
        self.break_seconds_spin = QSpinBox()
        self.break_seconds_spin.setRange(0, 59)
        self.break_seconds_spin.setValue(0)
        time_layout.addWidget(self.break_seconds_spin)
        time_layout.addWidget(QLabel("seconds"))
        time_layout.addStretch()
        break_group_layout.addLayout(time_layout)

        # Final sequence after break toggle and selector
        final_layout = QHBoxLayout()
        self.break_run_final_checkbox = QCheckBox("Run final sequence when time is hit")
        self.break_run_final_checkbox.setChecked(False)
        final_layout.addWidget(self.break_run_final_checkbox)
        final_layout.addWidget(QLabel("Final Sequence:"))
        self.break_final_sequence_combo = QComboBox()
        # Populate with available sequences names now; will be refreshed in update_ui_from_config
        try:
            seq_names = [s.get('name', 'Unnamed Sequence') for s in self.config.get('sequences', [])]
            self.break_final_sequence_combo.addItems(["(none)"] + seq_names)
        except Exception:
            self.break_final_sequence_combo.addItem("(none)")
        final_layout.addWidget(self.break_final_sequence_combo, 1)
        break_group_layout.addLayout(final_layout)

        break_group.setLayout(break_group_layout)
        break_layout.addWidget(break_group)
        break_layout.addStretch()

        # Toggle enabling of inputs based on checkbox
        def _set_break_inputs_enabled(enabled: bool):
            self.break_hours_spin.setEnabled(enabled)
            self.break_minutes_spin.setEnabled(enabled)
            self.break_seconds_spin.setEnabled(enabled)
            self.break_run_final_checkbox.setEnabled(enabled)
            self.break_final_sequence_combo.setEnabled(enabled)
        _set_break_inputs_enabled(False)
        self.break_enabled_checkbox.stateChanged.connect(
            lambda state: _set_break_inputs_enabled(state == Qt.CheckState.Checked.value)
        )

        # Add tabs to tab widget
        self.tab_widget.addTab(self.sequences_tab, "Sequences")
        # Scheduled Sequences tab
        self.scheduled_tab = QWidget()
        scheduled_layout = QVBoxLayout(self.scheduled_tab)
        # Controls: add schedule
        sched_controls = QHBoxLayout()
        self.add_sched_btn = QPushButton("Add Schedule")
        self.add_sched_btn.clicked.connect(self.add_schedule_row)
        sched_controls.addWidget(self.add_sched_btn)
        sched_controls.addStretch()
        scheduled_layout.addLayout(sched_controls)
        # Container for schedule rows
        self.schedules_container_widget = QWidget()
        self.schedules_container_layout = QVBoxLayout(self.schedules_container_widget)
        self.schedules_container_layout.setContentsMargins(0, 0, 0, 0)
        self.schedules_container_layout.setSpacing(8)
        scheduled_layout.addWidget(self.schedules_container_widget)
        scheduled_layout.addStretch()
        # Track rows
        self.schedule_rows = []
        self.tab_widget.addTab(self.scheduled_tab, "Scheduled Sequences")
        self.tab_widget.addTab(self.groups_tab, "Groups")
        self.tab_widget.addTab(self.failsafe_tab, "Failsafe")
        self.tab_widget.addTab(self.templates_tab, "Templates")
        self.tab_widget.addTab(self.template_tester_tab, "Template Tester")
        self.tab_widget.addTab(self.break_tab, "Break Settings")
        
        # Welcome screen
        welcome_widget = QWidget()
        # Default to Sequences tab on initial UI setup
        try:
            self.tab_widget.setCurrentWidget(self.sequences_tab)
        except Exception:
            pass
        welcome_layout = QVBoxLayout(welcome_widget)
        welcome_label = QLabel("Welcome to Image Detection Bot")
        welcome_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        welcome_layout.addWidget(welcome_label)
        welcome_layout.addStretch()
        
        # Template editor
        self.template_editor = QWidget()
        template_editor_layout = QVBoxLayout(self.template_editor)
        template_editor_layout.addWidget(QLabel("Template Editor"))
        
        # Sequence editor
        self.sequence_editor = QWidget()
        sequence_editor_layout = QVBoxLayout(self.sequence_editor)
        sequence_editor_layout.addWidget(QLabel("Sequence Editor"))
        
        # Add welcome widget to both stacks initially
        self.sequence_stack.addWidget(welcome_widget)
        self.template_stack.addWidget(QLabel("Select a template to edit"))

        # Initialize scheduler timer in MainWindow and update the indicator
        try:
            self.scheduler_timer = QTimer(self)
            self.scheduler_timer.setInterval(30000)  # check every 30 seconds
            self.scheduler_timer.timeout.connect(self.check_schedules)
            self.scheduler_timer.start()
            QTimer.singleShot(0, self.update_scheduler_indicator)
        except Exception as e:
            logger.debug(f"Scheduler init failed: {e}")

        # Initialize scheduler state
        self._scheduled_queue = []
        self._schedule_pending = None
        self._resume_after_scheduled = None
        self._current_run_is_scheduled = False

        # Simple IPC: poll for commands from web_server (ipc_command.json)
        try:
            self.ipc_timer = QTimer(self)
            self.ipc_timer.setInterval(400)
            self.ipc_timer.timeout.connect(self.handle_ipc_commands)
            self.ipc_timer.start()
        except Exception as e:
            logger.debug(f"IPC timer init failed: {e}")

    def add_schedule_row(self):
        """Add a new schedule row to the Scheduled Sequences tab."""
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(6, 6, 6, 6)
        enabled_chk = QCheckBox("Enabled")
        enabled_chk.setChecked(False)
        seq_combo = QComboBox()
        seq_names = [s.get('name', 'Unnamed Sequence') for s in self.config.get('sequences', [])]
        if not seq_names:
            seq_combo.addItem("(none)")
        else:
            for name in seq_names:
                seq_combo.addItem(name)
        time_edit = QTimeEdit()
        # Show 12-hour time with AM/PM, while storing 24-hour in config
        time_edit.setDisplayFormat("hh:mm AP")
        try:
            time_edit.setWrapping(True)
        except Exception:
            pass
        remove_btn = QPushButton("Remove")
        remove_btn.setToolTip("Remove this schedule")
        # assemble
        layout.addWidget(enabled_chk)
        layout.addWidget(QLabel("Sequence:"))
        layout.addWidget(seq_combo)
        layout.addWidget(QLabel("Time:"))
        layout.addWidget(time_edit)
        # Behavior toggles
        queue_chk = QCheckBox("Queue if busy")
        preempt_chk = QCheckBox("Preempt if busy")
        resume_chk = QCheckBox("Resume previous")
        queue_chk.setToolTip("Start scheduled sequence after current run finishes")
        preempt_chk.setToolTip("Stop current run and start scheduled sequence now")
        resume_chk.setToolTip("After scheduled finishes, resume the preempted run")
        layout.addWidget(queue_chk)
        layout.addWidget(preempt_chk)
        layout.addWidget(resume_chk)
        layout.addStretch()
        layout.addWidget(remove_btn)
        # Handle remove
        remove_btn.clicked.connect(lambda: self.remove_schedule_row(row))
        # Reflect changes immediately when user edits the row
        try:
            enabled_chk.toggled.connect(self._on_schedule_row_changed)
            seq_combo.currentIndexChanged.connect(self._on_schedule_row_changed)
            time_edit.timeChanged.connect(self._on_schedule_row_changed)
            queue_chk.toggled.connect(self._on_schedule_row_changed)
            preempt_chk.toggled.connect(self._on_schedule_row_changed)
            resume_chk.toggled.connect(self._on_schedule_row_changed)
        except Exception:
            pass
        # Track row components
        row._enabled_chk = enabled_chk
        row._seq_combo = seq_combo
        row._time_edit = time_edit
        row._queue_chk = queue_chk
        row._preempt_chk = preempt_chk
        row._resume_chk = resume_chk
        row._last_run_date = None
        self.schedules_container_layout.addWidget(row)
        self.schedule_rows.append(row)
        # Initial update so indicator and config reflect this new row
        try:
            self._on_schedule_row_changed()
        except Exception:
            pass

    def remove_schedule_row(self, row: QWidget):
        try:
            self.schedules_container_layout.removeWidget(row)
            row.deleteLater()
            if row in self.schedule_rows:
                self.schedule_rows.remove(row)
            # Persist changes and update indicator
            try:
                self.update_config_from_ui()
                self.save_config()
            except Exception:
                pass
            try:
                self.update_scheduler_indicator()
            except Exception:
                pass
        except Exception as e:
            logger.debug(f"remove_schedule_row failed: {e}")

        # No further UI resets needed
    
    def new_config(self):
        """Create a new configuration."""
        reply = QMessageBox.question(
            self,
            'New Configuration',
            'Are you sure you want to create a new configuration? Any unsaved changes will be lost.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.config = {
                "templates": {},
                "sequences": [],
                "groups": {},
                "break_settings": {"enabled": False, "max_runtime_seconds": 0}
            }
            self.update_ui_from_config()
            self.statusBar().showMessage("Created new configuration")
    
    def open_config(self):
        """Open a configuration file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open Configuration",
            "",
            "JSON Files (*.json);;All Files (*)"
        )
        
        if file_path:
            # Reuse the robust loader so paths are normalized and templates resolved
            try:
                self.load_config(file_path)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load configuration: {str(e)}")
    
    def save_config_as(self):
        """Save configuration to a new file."""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Configuration As",
            "",
            "JSON Files (*.json);;All Files (*)"
        )
        
        if file_path:
            if not file_path.lower().endswith('.json'):
                file_path += '.json'
            self.config_path = file_path
            self.save_config()
    
    def load_config(self, file_path: str = None):
        """Load configuration from file."""
        if file_path is None:
            file_path = self.config_path
            
        try:
            # Prevent auto-save hooks from firing during initial load
            try:
                self._suppress_auto_save = True
            except Exception:
                pass
            if os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    self.config = json.load(f)
                self.config_path = os.path.abspath(file_path)  # Store absolute path
                # Basic type normalization to prevent downstream UI errors
                try:
                    if not isinstance(self.config, dict):
                        raise ValueError("Config root must be an object (dict)")
                    # Ensure keys are of expected types
                    templates = self.config.get('templates') or {}
                    if not isinstance(templates, dict):
                        templates = {}
                    sequences = self.config.get('sequences') or []
                    if not isinstance(sequences, list):
                        sequences = []
                    groups = self.config.get('groups') or {}
                    if not isinstance(groups, dict):
                        groups = {}
                    break_settings = self.config.get('break_settings') or {"enabled": False, "max_runtime_seconds": 0}
                    if not isinstance(break_settings, dict):
                        break_settings = {"enabled": False, "max_runtime_seconds": 0}
                    self.config['templates'] = templates
                    self.config['sequences'] = sequences
                    self.config['groups'] = groups
                    self.config['break_settings'] = break_settings
                except Exception as e_norm:
                    logger.debug(f"Config normalization warning: {e_norm}")
                
                # Load search region if it exists
                if 'search_region' in self.config:
                    self.search_region = self.config['search_region']
                    if isinstance(self.search_region, (list, tuple)) and len(self.search_region) == 4:
                        x, y, w, h = self.search_region
                        if hasattr(self, 'region_status'):
                            self.region_status.setText(f"Region: ({x}, {y}, {w}, {h})")
                    else:
                        self.search_region = None
                        if hasattr(self, 'region_status'):
                            self.region_status.setText("Region: Full Screen")
                
                # Convert template paths to absolute paths for the bot
                if 'templates' in self.config:
                    updated_templates = {}
                    for name, path in self.config['templates'].items():
                        # Convert to absolute path if it's not already
                        if not os.path.isabs(path):
                            abs_path = os.path.normpath(os.path.join(os.path.dirname(self.config_path), path))
                            # Try to find the file in a few locations
                            possible_paths = [
                                abs_path,
                                os.path.join(self.script_dir, path),
                                os.path.join(self.images_dir, os.path.basename(path))
                            ]
                            
                            found = False
                            for possible_path in possible_paths:
                                if os.path.exists(possible_path):
                                    path = os.path.normpath(possible_path)
                                    found = True
                                    break
                            
                            if not found:
                                logger.warning(f"Template file not found: {path}")
                                # Keep original (possibly relative) path so the UI still shows the template
                                path = abs_path
                        
                        # Load the template into the bot if available on disk
                        if hasattr(self, 'bot') and os.path.exists(path):
                            success = self.bot.load_template(name, path)
                            if success:
                                updated_templates[name] = path
                            else:
                                logger.error(f"Failed to load template '{name}' from {path}")
                        else:
                            # Still include in config so the UI lists it (even if missing on disk)
                            updated_templates[name] = path
                    
                    self.config['templates'] = updated_templates
                
                # Update UI
                try:
                    self.update_ui_from_config()
                except Exception as e_ui:
                    logger.debug(f"update_ui_from_config skipped: {e_ui}")
                self.statusBar().showMessage(f"Loaded configuration from {os.path.basename(file_path)}")
            else:
                self.statusBar().showMessage("No configuration file found, using defaults")
        except Exception as e:
            logger.error(f"Failed to load configuration: {e}")
            # Fallback to defaults
            self.config = {
                "templates": {},
                "sequences": [],
                "groups": {},
                "break_settings": {"enabled": False, "max_runtime_seconds": 0}
            }
            try:
                self.update_ui_from_config()
            except Exception:
                pass
            self.statusBar().showMessage("Failed to load configuration — using defaults")
        finally:
            # Re-enable auto-save after finishing load
            try:
                self._suppress_auto_save = False
            except Exception:
                pass
    
    # Duplicate save_config removed; using the unified version defined later in the class.
    
    def toggle_region_selection(self, checked):
        """Toggle region selection mode."""
        if checked:
            dialog = ScreenCaptureDialog(self)
            if dialog.exec() == QDialog.DialogCode.Accepted:
                rect = dialog.get_capture_rect()
                if rect and rect.isValid():
                    self.search_region = (rect.x(), rect.y(), rect.width(), rect.height())
                    x, y, w, h = self.search_region
                    self.region_status.setText(f"Region: ({x}, {y}, {w}, {h})")
                    self.save_config()
            self.region_action.setChecked(False)
        else:
            if hasattr(self, 'region_selection'):
                self.region_selection.reject()

    def on_monitor_selected(self):
        """Set search_region to the selected monitor geometry or all monitors."""
        if not hasattr(self, 'monitor_combo'):
            return
        data = self.monitor_combo.currentData()
        if data == "ALL":
            # Use full-screen behavior for capture; update status with overall bounds
            self.search_region = None
            if getattr(self, 'monitor_regions', None):
                x0 = min(r[0] for r in self.monitor_regions)
                y0 = min(r[1] for r in self.monitor_regions)
                x1 = max(r[0] + r[2] for r in self.monitor_regions)
                y1 = max(r[1] + r[3] for r in self.monitor_regions)
                self.region_status.setText(f"Region: ALL ({x0}, {y0})..({x1}, {y1})")
            else:
                self.region_status.setText("Region: ALL")
            # Propagate to Template Tester
            if hasattr(self, 'template_tester'):
                self.template_tester.search_region = None
                if hasattr(self.template_tester, 'region_status'):
                    self.template_tester.region_status.setText("Region: Full Screen")
        elif isinstance(data, tuple) and len(data) == 4:
            self.search_region = data
            x, y, w, h = data
            self.region_status.setText(f"Region: ({x}, {y}, {w}, {h})")
            # Propagate to Template Tester
            if hasattr(self, 'template_tester'):
                self.template_tester.search_region = (x, y, w, h)
                if hasattr(self.template_tester, 'region_status'):
                    self.template_tester.region_status.setText(f"Region: X={x}, Y={y}, W={w}, H={h}")
        else:
            self.search_region = None
            self.region_status.setText("Region: Full Screen")
            # Propagate to Template Tester
            if hasattr(self, 'template_tester'):
                self.template_tester.search_region = None
                if hasattr(self.template_tester, 'region_status'):
                    self.template_tester.region_status.setText("Region: Full Screen")

    def show_monitor_info(self):
        try:
            dlg = MonitorInfoDialog(self)
            dlg.exec()
        except Exception as e:
            QMessageBox.warning(self, "Monitor Info", f"Failed to open monitor info: {e}")
    
    def update_ui_from_config(self):
        """Update UI elements from current config."""
        logger.info("Updating UI from config...")
        # Enable optional branch debug traces based on config
        try:
            branch_debug = bool(
                (self.config.get('debug', {}) or {}).get('branch_traces', False)
                or self.config.get('debug_branch_traces', False)
            )
            if hasattr(self, 'bot'):
                self.bot.branch_debug = branch_debug
            logger.info(f"Branch debug traces {'ENABLED' if branch_debug else 'DISABLED'}")
            try:
                if hasattr(self, 'branch_debug_checkbox') and self.branch_debug_checkbox:
                    self.branch_debug_checkbox.setChecked(branch_debug)
                
            except Exception:
                pass
        except Exception:
            pass
        
        # Safely reload templates into the bot without wiping on UI errors
        templates = self.config.get('templates', {})
        logger.info(f"Found {len(templates)} templates in config")
        prev_templates = {}
        try:
            if hasattr(self, 'bot'):
                # Preserve current templates in case reloading fails midway
                prev_templates = dict(getattr(self.bot, 'templates', {}) or {})
                logger.info("Clearing existing templates from bot before reload")
                self.bot.templates = {}
        except Exception as e:
            logger.debug(f"Failed to snapshot existing templates: {e}")

    def on_branch_debug_toggled(self, state):
        try:
            enabled = state == Qt.CheckState.Checked.value
        except Exception:
            enabled = bool(state)
        try:
            if hasattr(self, 'bot'):
                self.bot.branch_debug = enabled
            # Persist into config so Save writes it out
            dbg = self.config.get('debug') or {}
            dbg['branch_traces'] = enabled
            self.config['debug'] = dbg
            self.config['debug_branch_traces'] = enabled  # maintain backward compatibility
            logger.info(f"Branch debug toggle => {'ENABLED' if enabled else 'DISABLED'}")
        except Exception as e:
            logger.debug(f"on_branch_debug_toggled failed: {e}")
            
        # Update templates list and load them into the bot
        # Always repopulate the UI list first, then try loading each template into the bot
        templates = self.config.get('templates', {})
        try:
            if hasattr(self, 'templates_list'):
                self.templates_list.clear()
        except Exception:
            pass

        # Repopulate UI list regardless of any bot loading issues
        try:
            for name, path in templates.items():
                logger.info(f"Processing template: {name} -> {path}")
                if hasattr(self, 'templates_list'):
                    try:
                        self.templates_list.addItem(name)
                    except Exception:
                        pass
        except Exception as e:
            logger.debug(f"Failed repopulating templates list: {e}")

        # Load templates into bot individually to avoid aborting the whole UI refresh
        if hasattr(self, 'bot'):
            for name, path in templates.items():
                try:
                    if os.path.exists(path):
                        logger.info(f"Loading template '{name}' from {path}")
                        success = self.bot.load_template(name, path)
                        if success:
                            logger.info(f"Successfully loaded template '{name}'")
                        else:
                            logger.error(f"Failed to load template '{name}' from {path}")
                    else:
                        logger.error(f"Template file not found: {path}")
                except Exception as e:
                    logger.error(f"Error loading template '{name}' from {path}: {e}")
        
        # Update sequences list
        try:
            if hasattr(self, 'sequences_list'):
                self.sequences_list.clear()
                for seq in self.config.get('sequences', []):
                    self.sequences_list.addItem(seq.get('name', 'Unnamed Sequence'))
                # Auto-select first sequence for immediate usability
                if self.sequences_list.count() > 0 and not self.sequences_list.currentItem():
                    self.sequences_list.setCurrentRow(0)
        except Exception:
            pass

        # Update groups list
        if hasattr(self, 'groups_list'):
            self.groups_list.clear()
            for name in sorted(self.config.get('groups', {}).keys()):
                self.groups_list.addItem(name)
        # Default focus to Sequences tab after reload
        try:
            if hasattr(self, 'tab_widget') and hasattr(self, 'sequences_tab'):
                self.tab_widget.setCurrentWidget(self.sequences_tab)
        except Exception:
            pass
        
        # Update Template Tester's template list
        try:
            self.update_template_tester_templates()
        except Exception:
            pass

        # Refresh templates in active editors so new templates appear in dropdowns
        try:
            # Sequence editor refresh
            if hasattr(self, 'sequence_editor_widget') and self.sequence_editor_widget:
                self.sequence_editor_widget.templates = list(self.config.get('templates', {}).keys())
                # Also refresh available groups for group-call steps
                try:
                    self.sequence_editor_widget.update_groups(list(self.config.get('groups', {}).keys()))
                except Exception:
                    pass
                try:
                    if hasattr(self, 'sequences_list') and self.sequences_list:
                        current_seq_item = self.sequences_list.currentItem()
                        if current_seq_item:
                            self.on_sequence_selected(current_seq_item, None)
                except Exception:
                    pass
            # Group editor refresh
            if hasattr(self, 'group_editor_widget') and self.group_editor_widget:
                self.group_editor_widget.templates = list(self.config.get('templates', {}).keys())
                if hasattr(self, 'groups_list'):
                    current_group_item = self.groups_list.currentItem()
                    if current_group_item:
                        self.on_group_selected(current_group_item, None)
        except Exception as e:
            logger.debug(f"Editor template refresh skipped: {e}")

        # Update break settings UI
        br = self.config.get('break_settings', {"enabled": False, "max_runtime_seconds": 0})
        if hasattr(self, 'break_enabled_checkbox'):
            self.break_enabled_checkbox.setChecked(bool(br.get('enabled', False)))
        # Convert seconds to hours/minutes
        secs_total = int(br.get('max_runtime_seconds', 0) or 0)
        hrs = max(secs_total // 3600, 0)
        mins = max((secs_total % 3600) // 60, 0)
        secs = max(secs_total % 60, 0)
        if hasattr(self, 'break_hours_spin'):
            self.break_hours_spin.setValue(hrs)
        if hasattr(self, 'break_minutes_spin'):
            self.break_minutes_spin.setValue(mins)
        if hasattr(self, 'break_seconds_spin'):
            self.break_seconds_spin.setValue(secs)

        # Ensure templates list has a selection to enable template actions
        try:
            if hasattr(self, 'templates_list'):
                if self.templates_list.count() > 0 and not self.templates_list.currentItem():
                    self.templates_list.setCurrentRow(0)
                # Reflect delete button enabled state
                if hasattr(self, 'delete_template_btn'):
                    self.delete_template_btn.setEnabled(self.templates_list.currentItem() is not None)
        except Exception:
            pass
        # Enable/disable inputs based on enabled flag
        if hasattr(self, 'break_enabled_checkbox'):
            enabled = bool(br.get('enabled', False))
            self.break_hours_spin.setEnabled(enabled)
            self.break_minutes_spin.setEnabled(enabled)
            if hasattr(self, 'break_seconds_spin'):
                self.break_seconds_spin.setEnabled(enabled)
            if hasattr(self, 'break_run_final_checkbox'):
                self.break_run_final_checkbox.setEnabled(enabled)
                self.break_run_final_checkbox.setChecked(bool(br.get('run_final_after_break', False)))
            if hasattr(self, 'break_final_sequence_combo'):
                # Refresh sequence names
                try:
                    self.break_final_sequence_combo.clear()
                    seq_names = [s.get('name', 'Unnamed Sequence') for s in self.config.get('sequences', [])]
                    self.break_final_sequence_combo.addItems(["(none)"] + seq_names)
                    sel_name = br.get('final_sequence_name') or "(none)"
                    idx = self.break_final_sequence_combo.findText(sel_name)
                    if idx >= 0:
                        self.break_final_sequence_combo.setCurrentIndex(idx)
                except Exception:
                    pass
        
        # Update failsafe configuration
        failsafe_config = self.config.get('failsafe', {}) or {}
        enabled_flag = bool(failsafe_config.get('enabled', False))
        # If the user has the checkbox checked, prefer UI state over config to prevent immediate flip
        try:
            if hasattr(self, 'failsafe_enable_checkbox') and self.failsafe_enable_checkbox.isChecked():
                enabled_flag = True
        except Exception:
            pass
        # Reflect enabled flag into checkbox without re-entrant signals
        try:
            self.failsafe_enable_checkbox.blockSignals(True)
            self.failsafe_enable_checkbox.setChecked(enabled_flag)
            self.failsafe_enable_checkbox.blockSignals(False)
        except Exception:
            pass
        # Update UI state based on enabled flag
        self.toggle_failsafe_ui(enabled_flag)
        
        # Populate failsafe fields when enabled
        if enabled_flag:
            # Set template
            # Accept both 'template' (GUI) and 'template_name' (web API)
            template_name = failsafe_config.get('template') or failsafe_config.get('template_name')
            if template_name:
                index = self.failsafe_template_combo.findText(template_name)
                if index >= 0:
                    self.failsafe_template_combo.setCurrentIndex(index)
            # Set confidence
            confidence = failsafe_config.get('confidence', 0.8)
            self.failsafe_conf_spin.setValue(confidence)
            # Set region
            region = failsafe_config.get('region')
            if region:
                self.failsafe_region = region
                self.failsafe_region_label.setText(f"Region: {region[2]}x{region[3]} at ({region[0]}, {region[1]})")
            # Load failsafe sequence
            failsafe_sequence = failsafe_config.get('sequence', [])
            if hasattr(self, 'current_failsafe_editor'):
                try:
                    self.current_failsafe_editor.load_sequence(failsafe_sequence)
                except Exception:
                    pass
        # Ensure failsafe template combo reflects current templates
        self.refresh_failsafe_templates()
        # Populate scheduled sequences
        try:
            # Clear existing rows
            if hasattr(self, 'schedule_rows'):
                for r in list(self.schedule_rows):
                    self.remove_schedule_row(r)
            schedules = self.config.get('scheduled_sequences', [])
            for sch in schedules:
                self.add_schedule_row()
                row = self.schedule_rows[-1]
                # Set row values
                row._enabled_chk.setChecked(bool(sch.get('enabled', True)))
                seq_name = sch.get('sequence_name') or "(none)"
                idx = row._seq_combo.findText(seq_name)
                if idx >= 0:
                    row._seq_combo.setCurrentIndex(idx)
                # Time
                try:
                    from PyQt6.QtCore import QTime
                    hhmm = str(sch.get('time', '00:00')).strip()
                    # Support both 24-hour (HH:mm) and 12-hour (hh:mm AM/PM)
                    h, m = 0, 0
                    try:
                        from datetime import datetime
                        dt = datetime.strptime(hhmm, "%H:%M")
                        h, m = dt.hour, dt.minute
                    except Exception:
                        try:
                            from datetime import datetime
                            dt = datetime.strptime(hhmm, "%I:%M %p")
                            h, m = dt.hour, dt.minute
                        except Exception:
                            pass
                    row._time_edit.setTime(QTime(h, m))
                except Exception:
                    pass
                # Behavior toggles
                try:
                    if hasattr(row, '_queue_chk'):
                        row._queue_chk.setChecked(bool(sch.get('queue_if_busy', False)))
                    if hasattr(row, '_preempt_chk'):
                        row._preempt_chk.setChecked(bool(sch.get('preempt_if_busy', False)))
                    if hasattr(row, '_resume_chk'):
                        row._resume_chk.setChecked(bool(sch.get('resume_previous', False)))
                except Exception:
                    pass
                # Last run date
                row._last_run_date = sch.get('last_run_date')
        except Exception as e:
            logger.debug(f"Failed to populate scheduled sequences: {e}")

    def refresh_break_sequences(self):
        """Refresh the 'Final Sequence' dropdown in Break Settings with current sequences."""
        try:
            if hasattr(self, 'break_final_sequence_combo'):
                current_text = self.break_final_sequence_combo.currentText()
                self.break_final_sequence_combo.clear()
                seq_names = [s.get('name', 'Unnamed Sequence') for s in self.config.get('sequences', [])]
                self.break_final_sequence_combo.addItems(["(none)"] + seq_names)
                # Try to preserve selection if it still exists
                if current_text and self.break_final_sequence_combo.findText(current_text) >= 0:
                    self.break_final_sequence_combo.setCurrentText(current_text)
        except Exception as e:
            logger.debug(f"refresh_break_sequences failed: {e}")
    
    def update_config_from_ui(self):
        """Update config from UI elements."""
        # Update templates (only if an editor widget with get_data is active)
        try:
            template_data = None
            if hasattr(self, 'template_editor') and hasattr(self.template_editor, 'get_data'):
                template_data = self.template_editor.get_data()
            elif hasattr(self, 'template_stack'):
                current_widget = self.template_stack.currentWidget()
                if current_widget is not None and hasattr(current_widget, 'get_data'):
                    template_data = current_widget.get_data()
            if template_data and template_data.get('name'):
                self.config.setdefault('templates', {})[template_data['name']] = template_data.get('path', '')
        except Exception as e:
            logger.debug(f"Skipping template update from UI: {e}")
        
        # Update sequences and preserve loop fields
        if hasattr(self, 'sequence_editor_widget'):
            # Skip sequence write-back during editor rebuild to avoid wiping steps
            if not getattr(self, '_rebuilding_sequence_editor', False):
                sequence_data = self.sequence_editor_widget.get_sequence_data()
                name = sequence_data.get('name')
                steps = sequence_data.get('steps')
                if name:
                    # Preserve loop settings from UI
                    ui_loop = self.loop_checkbox.isChecked() if hasattr(self, 'loop_checkbox') else None
                    ui_loop_count = self.loop_count_spin.value() if hasattr(self, 'loop_count_spin') else None
                    # Find existing sequence
                    existing_index = -1
                    existing_seq = None
                    for i, seq in enumerate(self.config.get('sequences', [])):
                        if seq.get('name') == name:
                            existing_index = i
                            existing_seq = seq
                            break
                    # If editor reports no steps but config has steps, do not overwrite
                    if not isinstance(steps, list) or (steps == [] and existing_seq and isinstance(existing_seq.get('steps'), list) and len(existing_seq.get('steps')) > 0):
                        pass  # Keep existing steps
                    else:
                        merged = sequence_data.copy()
                        merged['loop'] = ui_loop if ui_loop is not None else (existing_seq.get('loop', False) if existing_seq else False)
                        merged['loop_count'] = ui_loop_count if ui_loop_count is not None else (existing_seq.get('loop_count', 1) if existing_seq else 1)
                        if existing_index >= 0:
                            self.config['sequences'][existing_index] = merged
                        else:
                            self.config.setdefault('sequences', []).append(merged)
        
        # Update failsafe configuration
        failsafe_config = self.get_failsafe_config()
        if failsafe_config:
            self.config['failsafe'] = failsafe_config
        else:
            self.config.pop('failsafe', None)  # Remove failsafe config if disabled

        # Update break settings
        if hasattr(self, 'break_enabled_checkbox'):
            enabled = self.break_enabled_checkbox.isChecked()
            hrs = self.break_hours_spin.value() if hasattr(self, 'break_hours_spin') else 0
            mins = self.break_minutes_spin.value() if hasattr(self, 'break_minutes_spin') else 0
            secs_extra = self.break_seconds_spin.value() if hasattr(self, 'break_seconds_spin') else 0
            secs = int(hrs) * 3600 + int(mins) * 60 + int(secs_extra)
            self.config['break_settings'] = {
                'enabled': bool(enabled),
                'max_runtime_seconds': int(secs),
                'run_final_after_break': bool(getattr(self, 'break_run_final_checkbox', None) and self.break_run_final_checkbox.isChecked()),
                'final_sequence_name': (self.break_final_sequence_combo.currentText() if hasattr(self, 'break_final_sequence_combo') else "(none)")
            }

        # Update scheduled sequences from UI
        try:
            schedules = []
            if hasattr(self, 'schedule_rows'):
                for row in self.schedule_rows:
                    seq_name = row._seq_combo.currentText()
                    enabled = row._enabled_chk.isChecked()
                    t = row._time_edit.time()
                    # Persist as 12-hour AM/PM string for user-friendly readability
                    h24 = int(t.hour())
                    m = int(t.minute())
                    ap = "AM" if h24 < 12 else "PM"
                    h12 = h24 % 12
                    if h12 == 0:
                        h12 = 12
                    hhmm = f"{h12:02d}:{m:02d} {ap}"
                    queue_if_busy = bool(getattr(row, '_queue_chk', None) and row._queue_chk.isChecked())
                    preempt_if_busy = bool(getattr(row, '_preempt_chk', None) and row._preempt_chk.isChecked())
                    resume_previous = bool(getattr(row, '_resume_chk', None) and row._resume_chk.isChecked())
                    if preempt_if_busy:
                        queue_if_busy = False
                    schedules.append({
                        'enabled': bool(enabled),
                        'sequence_name': seq_name,
                        'time': hhmm,
                        'queue_if_busy': queue_if_busy,
                        'preempt_if_busy': preempt_if_busy,
                        'resume_previous': resume_previous,
                        'last_run_date': row._last_run_date
                    })
            self.config['scheduled_sequences'] = schedules
        except Exception as e:
            logger.debug(f"Failed to update scheduled sequences from UI: {e}")

        # Update groups if an editor is open
        try:
            if hasattr(self, 'group_editor_widget') and self.group_editor_widget:
                data = self.group_editor_widget.get_sequence_data()
                name = data.get('name')
                steps = data.get('steps', [])
                if name:
                    # Only update groups that still exist or are currently selected
                    exists_in_config = name in (self.config.get('groups', {}) or {})
                    selected_name = None
                    try:
                        if hasattr(self, 'groups_list') and self.groups_list.currentItem():
                            selected_name = self.groups_list.currentItem().text()
                    except Exception:
                        selected_name = None
                    if exists_in_config or selected_name == name:
                        self.config.setdefault('groups', {})[name] = steps
        except Exception as e:
            logger.debug(f"Skipping group update from UI: {e}")

    def add_group(self):
        """Add a new group."""
        try:
            # Prompt for name
            default_name = f"Group_{len(self.config.get('groups', {}) ) + 1}"
            name, ok = QInputDialog.getText(self, "Add Group", "Enter group name:", QLineEdit.EchoMode.Normal, default_name)
            if not ok:
                return
            name = name.strip()
            if not name:
                QMessageBox.warning(self, "Error", "Group name cannot be empty")
                return
            if name in self.config.get('groups', {}):
                QMessageBox.warning(self, "Error", f"Group '{name}' already exists")
                return
            # Create empty group
            self.config.setdefault('groups', {})[name] = []
            # Update list
            self.groups_list.addItem(name)
            # Select and open for editing
            items = self.groups_list.findItems(name, Qt.MatchFlag.MatchExactly)
            if items:
                self.groups_list.setCurrentItem(items[0])
            self.save_config()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to add group: {e}")

    def remove_group(self):
        """Remove selected group."""
        current_item = self.groups_list.currentItem() if hasattr(self, 'groups_list') else None
        if not current_item:
            return
        name = current_item.text()
        reply = QMessageBox.question(self, 'Remove Group', f"Delete group '{name}'?", QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No)
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            # Remove from config
            if name in self.config.get('groups', {}):
                del self.config['groups'][name]
            # Update sequences referencing this group
            for seq in self.config.get('sequences', []):
                for step in seq.get('steps', []):
                    if isinstance(step, dict) and step.get('call_group') == name:
                        step['call_group'] = ''
            # Update UI
            row = self.groups_list.row(current_item)
            self.groups_list.takeItem(row)
            # Clear editor
            if hasattr(self, 'group_stack'):
                self.group_stack.setCurrentIndex(0) if self.group_stack.count() > 0 else None
            # Drop stale editor reference to avoid resurrecting deleted groups during config sync
            try:
                self.group_editor_widget = None
            except Exception:
                pass
            self.save_config()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to remove group: {e}")

    def rename_group(self):
        """Rename selected group and update references."""
        current_item = self.groups_list.currentItem() if hasattr(self, 'groups_list') else None
        if not current_item:
            return
        old_name = current_item.text()
        new_name, ok = QInputDialog.getText(self, "Rename Group", "Enter new group name:", QLineEdit.EchoMode.Normal, old_name)
        if not ok:
            return
        new_name = new_name.strip()
        if not new_name or new_name == old_name:
            return
        if new_name in self.config.get('groups', {}):
            QMessageBox.warning(self, "Error", f"Group '{new_name}' already exists")
            return
        try:
            # Move group steps to new name
            steps = self.config.get('groups', {}).pop(old_name, [])
            self.config.setdefault('groups', {})[new_name] = steps
            # Update references in sequences
            for seq in self.config.get('sequences', []):
                for step in seq.get('steps', []):
                    if isinstance(step, dict) and step.get('call_group') == old_name:
                        step['call_group'] = new_name
            # Update UI selection
            current_item.setText(new_name)
            # Refresh editor name field if open
            if hasattr(self, 'group_editor_widget') and self.group_editor_widget and hasattr(self.group_editor_widget, 'name_edit'):
                self.group_editor_widget.name_edit.setText(new_name)
            self.save_config()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to rename group: {e}")

    def on_group_selected(self, current, previous):
        """Open the selected group in the editor."""
        has_selection = bool(current)
        if hasattr(self, 'rename_group_btn'):
            self.rename_group_btn.setEnabled(has_selection)
        if hasattr(self, 'remove_group_btn'):
            self.remove_group_btn.setEnabled(has_selection)
        if not current:
            return
        name = current.text()
        steps = self.config.get('groups', {}).get(name, [])
        # Build sequence_data so we can reuse SequenceEditor for groups
        sequence_data = {"name": name, "steps": steps}
        try:
            # Create editor if needed
            self.group_editor_widget = SequenceEditor(sequence_data, list(self.config.get('templates', {}).keys()), self, groups=list(self.config.get('groups', {}).keys()))
            if hasattr(self, 'group_stack'):
                # Clear existing editors to avoid duplicates
                while self.group_stack.count() > 0:
                    w = self.group_stack.widget(0)
                    self.group_stack.removeWidget(w)
                    w.deleteLater()
                self.group_stack.addWidget(self.group_editor_widget)
                self.group_stack.setCurrentWidget(self.group_editor_widget)
            # Switch to Groups tab to edit
            if hasattr(self, 'tab_widget'):
                idx = self.tab_widget.indexOf(self.groups_tab)
                if idx != -1:
                    self.tab_widget.setCurrentIndex(idx)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to open group: {e}")
    
    def save_current_template(self):
        """Save the currently edited template and load it into the bot."""
        if not hasattr(self, 'template_editor'):
            return False
            
        template_data = self.template_editor.get_data()
        if not template_data['name'] or not template_data['path']:
            return False
            
        try:
            # Get absolute path to the template
            template_path = template_data['path']
            if not os.path.isabs(template_path):
                template_path = os.path.join(os.path.dirname(os.path.abspath(self.config_path)), template_path)
            
            # Ensure the template file exists
            if not os.path.exists(template_path):
                self.statusBar().showMessage(f"Error: Template file not found: {template_path}")
                return False
                
            # Convert to relative path for config
            rel_path = os.path.relpath(template_path, os.path.dirname(os.path.abspath(self.config_path)))
            
            # Load the template into the bot
            if hasattr(self, 'bot'):
                success = self.bot.load_template(template_data['name'], template_path)
                if not success:
                    self.statusBar().showMessage(f"Failed to load template '{template_data['name']}' into bot")
                    return False
            
            # Store old name for updating references if needed
            old_name = None
            current_item = self.templates_list.currentItem()
            if current_item and current_item.text() != template_data['name']:
                old_name = current_item.text()
            
            # Update config with relative path
            self.config.setdefault('templates', {})[template_data['name']] = rel_path
            
            # Remove old entry if name was changed
            if old_name and old_name in self.config.get('templates', {}):
                del self.config['templates'][old_name]
            
            # Update list if name changed
            if current_item and current_item.text() != template_data['name']:
                current_item.setText(template_data['name'])
                
                # If the template name was changed, remove the old entry
                for i in range(self.templates_list.count()):
                    item = self.templates_list.item(i)
                    if item != current_item and item.text() == template_data['name']:
                        self.templates_list.takeItem(i)
                        break
            
            # If this is a new template, add it to the list
            if not any(self.templates_list.item(i).text() == template_data['name'] 
                      for i in range(self.templates_list.count())):
                self.templates_list.addItem(template_data['name'])
            
            # Update any sequences that reference this template if the name changed
            if old_name and old_name != template_data['name']:
                for seq in self.config.get('sequences', []):
                    for step in seq.get('steps', []):
                        if step.get('find') == old_name:
                            step['find'] = template_data['name']
            
            # Save the updated config
            if self.save_config():
                self.statusBar().showMessage(f"Template '{template_data['name']}' saved and loaded")
                
                # Update the sequence editor's template list and refresh UI
                if hasattr(self, 'sequence_editor_widget'):
                    self.sequence_editor_widget.templates = list(self.config.get('templates', {}).keys())
                    # Force refresh the current sequence editor
                    if hasattr(self, 'current_sequence_widget'):
                        self.on_sequence_selected(self.sequences_list.currentItem(), None)
                
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"Error saving template: {str(e)}", exc_info=True)
            self.statusBar().showMessage(f"Error saving template: {str(e)}")
            return False
    
    def save_current_sequence(self):
        """Save the currently edited sequence."""
        if not hasattr(self, 'sequence_editor_widget') or not hasattr(self, 'sequences_list'):
            return False
            
        current_item = self.sequences_list.currentItem()
        if not current_item:
            return False
            
        sequence_name = current_item.text()
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == sequence_name), None)
        
        if sequence:
            # Update sequence data from editor
            sequence_data = self.sequence_editor_widget.get_sequence_data()
            sequence.update(sequence_data)
            
            # Update loop settings
            if hasattr(self, 'loop_checkbox'):
                sequence['loop'] = self.loop_checkbox.isChecked()
                if hasattr(self, 'loop_count_spin'):
                    sequence['loop_count'] = self.loop_count_spin.value()
            
            # Save the config
            return self.save_config()
            
        return False
    
    def update_template_tester_templates(self):
        """Update the template list in the TemplateTester widget."""
        if hasattr(self, 'template_tester'):
            self.template_tester.templates = self.config.get('templates', {}).copy()
            # Clear the current dropdown
            self.template_tester.template_combo.clear()
            self.template_tester.template_combo.addItem("Select a template...", None)
            # Repopulate with updated templates
            for name, path in sorted(self.template_tester.templates.items()):
                self.template_tester.template_combo.addItem(name, path)
    
    def add_template(self, name: str = None, path: str = None):
        """Add a new template."""
        try:
            if name is None:
                name = f"template_{len(self.config.get('templates', {})) + 1}"
            
            # Create and show the template dialog
            dialog = QDialog(self)
            dialog.setWindowTitle("Add New Template")
            dialog.setLayout(QVBoxLayout())
            
            # Create the preview widget
            preview = TemplatePreview(name, path or "", dialog)  # Pass dialog as parent
            
            # Add widgets to dialog
            dialog.layout().addWidget(preview)
            
            # Button layout
            btn_box = QHBoxLayout()
            
            capture_btn = QPushButton("Capture from Screen")
            capture_btn.clicked.connect(preview.capture_from_screen)
            
            select_btn = QPushButton("Select from File")
            select_btn.clicked.connect(preview.select_from_file)
            
            btn_box.addWidget(capture_btn)
            btn_box.addWidget(select_btn)
            btn_box.addStretch()
            
            # Dialog buttons
            button_box = QDialogButtonBox(
                QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel,
                Qt.Orientation.Horizontal, dialog
            )
            button_box.accepted.connect(dialog.accept)
            button_box.rejected.connect(dialog.reject)
            
            dialog.layout().addLayout(btn_box)
            dialog.layout().addWidget(button_box)
            
            # Show the dialog
            if dialog.exec() == QDialog.DialogCode.Accepted:
                name = preview.name_edit.text().strip()
                path = preview.path_edit.text().strip()
                
                if not name or not path:
                    QMessageBox.warning(self, "Error", "Template name and image path are required")
                    return False
                    
                if not os.path.exists(path):
                    QMessageBox.warning(self, "Error", f"Image file not found: {path}")
                    return False
                
                # Ensure the images directory exists
                os.makedirs(self.images_dir, exist_ok=True)
                
                # Determine the destination path in the images directory
                dest_filename = f"{name}{os.path.splitext(path)[1]}"
                dest_path = os.path.join(self.images_dir, dest_filename)
                
                # If the file is not already in the images directory, copy it there
                if os.path.normpath(os.path.abspath(path)) != os.path.normpath(dest_path):
                    try:
                        # Check if destination file exists and is different from source
                        if os.path.exists(dest_path):
                            # If files are the same, just use the existing one
                            if os.path.samefile(path, dest_path):
                                path = dest_path
                            else:
                                # If different, ask user if they want to overwrite
                                reply = QMessageBox.question(
                                    self, 'File Exists',
                                    f"A template with the name '{name}' already exists. Overwrite?",
                                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                    QMessageBox.StandardButton.No
                                )
                                if reply == QMessageBox.StandardButton.Yes:
                                    shutil.copy2(path, dest_path)
                                    path = dest_path
                                else:
                                    return False
                        else:
                            shutil.copy2(path, dest_path)
                            path = dest_path
                    except Exception as e:
                        QMessageBox.warning(self, "Error", f"Failed to copy template to images directory: {str(e)}")
                        return False
                else:
                    # If the file is already in the images directory, just use it as is
                    path = dest_path
                
                # Add to config with relative path
                rel_path = os.path.relpath(path, os.path.dirname(self.config_path))
                self.config.setdefault('templates', {})[name] = rel_path
                
                # Load the template into the bot using the absolute path
                abs_path = os.path.abspath(os.path.join(os.path.dirname(self.config_path), rel_path))
                if hasattr(self, 'bot') and os.path.exists(abs_path):
                    success = self.bot.load_template(name, abs_path)
                    if not success:
                        QMessageBox.warning(self, "Error", f"Failed to load template '{name}' into bot")
                        return False
                
                # Add to UI
                self.templates_list.addItem(name)
                self.templates_list.setCurrentRow(self.templates_list.count() - 1)
                
                # Update sequence editor's template list if it exists
                if hasattr(self, 'sequence_editor_widget'):
                    self.sequence_editor_widget.templates = list(self.config.get('templates', {}).keys())
                    # Force refresh the current step editor if it exists
                    if hasattr(self, 'current_sequence_widget'):
                        self.on_sequence_selected(self.sequences_list.currentItem(), None)
                # Update groups editor's template list if it exists
                if hasattr(self, 'group_editor_widget') and self.group_editor_widget:
                    self.group_editor_widget.templates = list(self.config.get('templates', {}).keys())
                    if hasattr(self, 'groups_list'):
                        current_group_item = self.groups_list.currentItem()
                        if current_group_item:
                            self.on_group_selected(current_group_item, None)
                
                # Save config
                if not self.save_config():
                    return False
                
                # Update the Template Tester's template list
                self.update_template_tester_templates()
                
                # Show the template in the editor
                self.on_template_selected(self.templates_list.currentItem(), None)
                
                return True
                
            return False
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to add template: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def delete_template(self):
        """Delete the currently selected template."""
        current_item = self.templates_list.currentItem()
        if not current_item:
            return
            
        template_name = current_item.text()
        
        # Check if template is used in any sequence
        used_in_sequences = []
        for seq in self.config.get('sequences', []):
            for step in seq.get('steps', []):
                if step.get('find') == template_name:
                    used_in_sequences.append(seq.get('name', 'Unnamed Sequence'))
                    break
        
        if used_in_sequences:
            seq_list = '\n- ' + '\n- '.join(used_in_sequences)
            QMessageBox.warning(
                self, 
                'Cannot Delete Template',
                f'Template "{template_name}" is used in the following sequences and cannot be deleted:\n{seq_list}\n\nPlease remove it from these sequences first.',
                QMessageBox.StandardButton.Ok
            )
            return
            
        reply = QMessageBox.question(
            self,
            'Confirm Delete',
            f'Are you sure you want to delete the template "{template_name}"?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            try:
                # Remove the template file if it exists in the images directory
                if template_name in self.config.get('templates', {}):
                    template_path = self.config['templates'][template_name]
                    if os.path.isabs(template_path) and os.path.exists(template_path):
                        try:
                            os.remove(template_path)
                        except Exception as e:
                            logger.warning(f"Could not delete template file {template_path}: {e}")
                
                # Remove from config
                if 'templates' in self.config and template_name in self.config['templates']:
                    del self.config['templates'][template_name]
                
                # Remove from UI
                row = self.templates_list.row(current_item)
                self.templates_list.takeItem(row)
                
                # Clear the template editor if this was the selected template
                if hasattr(self, 'current_template_widget'):
                    self.template_stack.removeWidget(self.current_template_widget)
                    self.current_template_widget.deleteLater()
                    del self.current_template_widget
                    # Reset template stack to default view
                    while self.template_stack.count() > 0:
                        widget = self.template_stack.widget(0)
                        self.template_stack.removeWidget(widget)
                        widget.deleteLater()
                    self.template_stack.addWidget(QLabel("Select a template to edit"))
                
                # Remove from bot's template cache
                if hasattr(self, 'bot') and hasattr(self.bot, 'templates') and template_name in self.bot.templates:
                    del self.bot.templates[template_name]
                
                # Update sequence editor's template list if it exists
                if hasattr(self, 'sequence_editor_widget'):
                    self.sequence_editor_widget.templates = list(self.config.get('templates', {}).keys())
                    # Re-open current sequence to rebuild editors
                    if hasattr(self, 'sequences_list'):
                        current_seq_item = self.sequences_list.currentItem()
                        if current_seq_item:
                            self.on_sequence_selected(current_seq_item, None)
                # Update groups editor's template list if it exists
                if hasattr(self, 'group_editor_widget') and self.group_editor_widget:
                    self.group_editor_widget.templates = list(self.config.get('templates', {}).keys())
                    if hasattr(self, 'groups_list'):
                        current_group_item = self.groups_list.currentItem()
                        if current_group_item:
                            self.on_group_selected(current_group_item, None)

                # Save changes
                self.save_config()
                self.statusBar().showMessage(f"Template '{template_name}' deleted")
                
            except Exception as e:
                logger.error(f"Error deleting template: {e}", exc_info=True)
                QMessageBox.critical(
                    self,
                    'Error',
                    f'Failed to delete template: {str(e)}',
                    QMessageBox.StandardButton.Ok
                )
    
    def rename_sequence(self):
        """Rename the currently selected sequence."""
        current_item = self.sequences_list.currentItem()
        if not current_item:
            return
            
        old_name = current_item.text()
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == old_name), None)
        if not sequence:
            return
            
        # Show input dialog for new name
        new_name, ok = QInputDialog.getText(
            self, 
            "Rename Sequence", 
            "Enter new sequence name:",
            QLineEdit.EchoMode.Normal,
            old_name
        )
        
        if not ok or not new_name.strip() or new_name == old_name:
            return  # User cancelled or entered empty/same name
            
        # Check if name already exists
        existing_names = [s.get('name') for s in self.config.get('sequences', [])]
        if new_name in existing_names:
            QMessageBox.warning(self, "Error", f"A sequence named '{new_name}' already exists.")
            return
            
        # Update sequence name in config
        sequence['name'] = new_name
        
        # Update UI
        current_item.setText(new_name)
        
        # Save changes
        if self.save_config():
            self.statusBar().showMessage(f"Sequence renamed to '{new_name}'")
    
    def duplicate_sequence(self):
        """Create a copy of the currently selected sequence."""
        current_item = self.sequences_list.currentItem()
        if not current_item:
            return
            
        sequence_name = current_item.text()
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == sequence_name), None)
        if not sequence:
            return
            
        # Create a deep copy of the sequence
        import copy
        new_sequence = copy.deepcopy(sequence)
        
        # Generate a unique name for the copy
        base_name = f"{sequence_name} (Copy)"
        new_name = base_name
        counter = 1
        
        existing_names = [s.get('name') for s in self.config.get('sequences', [])]
        while new_name in existing_names:
            new_name = f"{base_name} {counter}"
            counter += 1
            
        new_sequence['name'] = new_name
        
        # Add the new sequence to config
        self.config['sequences'].append(new_sequence)
        
        # Update UI
        self.sequences_list.addItem(new_name)
        self.sequences_list.setCurrentRow(self.sequences_list.count() - 1)
        
        # Save changes
        if self.save_config():
            self.statusBar().showMessage(f"Created copy: '{new_name}'")
            # Refresh break settings final sequence dropdown
            if hasattr(self, 'refresh_break_sequences'):
                self.refresh_break_sequences()
    
    def remove_sequence(self):
        """Remove the currently selected sequence."""
        current_item = self.sequences_list.currentItem()
        if not current_item:
            return
            
        sequence_name = current_item.text()
        
        reply = QMessageBox.question(
            self,
            'Confirm Delete',
            f'Are you sure you want to delete the sequence "{sequence_name}"?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            # Remove from config
            self.config['sequences'] = [s for s in self.config.get('sequences', []) 
                                     if s.get('name') != sequence_name]
            
            # Remove from UI
            row = self.sequences_list.row(current_item)
            self.sequences_list.takeItem(row)
            
            # Clear the sequence editor if this was the selected sequence
            if hasattr(self, 'sequence_editor_widget') and hasattr(self.sequence_editor_widget, 'sequence_data') and \
               self.sequence_editor_widget.sequence_data.get('name') == sequence_name:
                if hasattr(self, 'main_content_layout') and hasattr(self, 'current_sequence_widget'):
                    self.main_content_layout.removeWidget(self.current_sequence_widget)
                if hasattr(self, 'current_sequence_widget'):
                    self.current_sequence_widget.deleteLater()
                if hasattr(self, 'sequence_editor_widget'):
                    del self.sequence_editor_widget
                if hasattr(self, 'current_sequence_widget'):
                    del self.current_sequence_widget
            
            # Save changes
            self.save_config()
            self.statusBar().showMessage(f"Sequence '{sequence_name}' deleted")
            # Refresh break settings final sequence dropdown
            if hasattr(self, 'refresh_break_sequences'):
                self.refresh_break_sequences()
    
    def add_sequence(self):
        """Add a new sequence."""
        try:
            print("DEBUG: Entering add_sequence method")  # Debug print
            
            # Ensure config exists and has sequences list
            if not hasattr(self, 'config'):
                print("DEBUG: No config found, initializing")  # Debug print
                self.config = {'sequences': []}
            
            if 'sequences' not in self.config:
                print("DEBUG: Initializing empty sequences list")  # Debug print
                self.config['sequences'] = []
            
            # Create default name
            default_name = f"sequence_{len(self.config['sequences']) + 1}"
            print(f"DEBUG: Default name: {default_name}")  # Debug print
            
            # Show input dialog
            print("DEBUG: Showing input dialog")  # Debug print
            name, ok = QInputDialog.getText(
                self, 
                "New Sequence", 
                "Enter sequence name:",
                QLineEdit.EchoMode.Normal,
                default_name
            )
            print(f"DEBUG: Got input - ok: {ok}, name: {name}")  # Debug print
            
            if not ok:
                print("DEBUG: User cancelled")  # Debug print
                return
                
            name = name.strip()
            if not name:
                print("DEBUG: Empty name provided")  # Debug print
                QMessageBox.warning(self, "Error", "Sequence name cannot be empty")
                return
            
            print(f"DEBUG: Processing name: {name}")  # Debug print
            
            # Check if name already exists
            existing_sequences = []
            for seq in self.config.get('sequences', []):
                if isinstance(seq, dict):
                    existing_sequences.append(seq.get('name', ''))
            
            print(f"DEBUG: Existing sequences: {existing_sequences}")  # Debug print
            
            # Check if name already exists in sequences
            if name in existing_sequences:
                print(f"DEBUG: Name already exists: {name}")  # Debug print
                QMessageBox.warning(self, "Error", f"A sequence named '{name}' already exists.")
                return
            
            # Create new sequence with default loop settings
            new_sequence = {
                'name': name,
                'steps': [],
                'loop': False,
                'loop_count': 1
            }
            
            # Add to config
            self.config['sequences'].append(new_sequence)
            
            # Update UI
            print("DEBUG: Updating UI")  # Debug print
            if not hasattr(self, 'sequences_list'):
                print("DEBUG: sequences_list not found, initializing")  # Debug print
                self.sequences_list = QListWidget()
                
            self.sequences_list.addItem(name)
            self.sequences_list.setCurrentRow(self.sequences_list.count() - 1)
            # Refresh break settings final sequence dropdown
            if hasattr(self, 'refresh_break_sequences'):
                self.refresh_break_sequences()
            
            # Save the config
            print("DEBUG: Saving config")  # Debug print
            if not self.save_config():
                print("DEBUG: Failed to save config")  # Debug print
                QMessageBox.warning(self, "Error", "Failed to save configuration.")
                
            print(f"DEBUG: Successfully added sequence: {name}")  # Debug print
                
        except Exception as e:
            import traceback
            error_msg = f"Error in add_sequence: {str(e)}\n\n{traceback.format_exc()}"
            print(f"ERROR: {error_msg}")  # Debug print
            QMessageBox.critical(self, "Error", f"Failed to add sequence: {str(e)}\n\n{error_msg}")
    
    def on_template_selected(self, current, previous):
        """Handle template selection change."""
        # Enable/disable delete button based on selection
        self.delete_template_btn.setEnabled(current is not None)
        
        if current is None:
            return
            
        template_name = current.text()
        if template_name in self.config.get('templates', {}):
            # Clear the template stack first
            while self.template_stack.count() > 0:
                widget = self.template_stack.widget(0)
                self.template_stack.removeWidget(widget)
                if widget and widget.isWidgetType() and widget.parent() is None:
                    widget.deleteLater()
            
            # Create new template editor
            template_path = self.config['templates'][template_name]
            self.template_editor = TemplatePreview(template_name, template_path)
            
            # Create container widget
            self.current_template_widget = QWidget()
            layout = QVBoxLayout(self.current_template_widget)
            layout.addWidget(self.template_editor)
            
            # Add save button
            save_btn = QPushButton("Save Template")
            save_btn.clicked.connect(self.save_current_template)
            layout.addWidget(save_btn)
            
            # Add to template stack and show
            self.template_stack.addWidget(self.current_template_widget)
            self.template_stack.setCurrentWidget(self.current_template_widget)
            
            # Make sure we're on the templates tab
            try:
                # Avoid tab switching during config/UI reloads
                if not getattr(self, '_suppress_tab_switch', False) and not getattr(self, '_suppress_auto_save', False):
                    self.tab_widget.setCurrentWidget(self.templates_tab)
            except Exception:
                pass
    
    def on_sequence_selected(self, current, previous):
        """Handle sequence selection change."""
        # Enable/disable action buttons based on selection
        has_selection = current is not None
        if hasattr(self, 'duplicate_sequence_btn'):
            self.duplicate_sequence_btn.setEnabled(has_selection)
        if hasattr(self, 'rename_sequence_btn'):
            self.rename_sequence_btn.setEnabled(has_selection)
        if hasattr(self, 'remove_sequence_btn'):
            self.remove_sequence_btn.setEnabled(has_selection)
            
        if not current:
            return
            
        sequence_name = current.text()
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == sequence_name), None)
        
        if sequence:
            # Mark that we are rebuilding the editor to avoid config writes with partial data
            try:
                self._rebuilding_sequence_editor = True
            except Exception:
                pass
            # Temporarily suppress autosave while rebuilding editor
            try:
                self._suppress_auto_save = True
            except Exception:
                pass

            # Update loop controls state
            if hasattr(self, 'loop_checkbox'):
                loop_enabled = bool(sequence.get('loop', False))
                self.loop_checkbox.setChecked(loop_enabled)
                if hasattr(self, 'loop_count_spin'):
                    self.loop_count_spin.setValue(int(sequence.get('loop_count', 1)))
                    self.loop_count_spin.setEnabled(loop_enabled)
            
            # Always build a fresh SequenceEditor for the selected sequence to avoid stale state
            new_editor = SequenceEditor(
                sequence.copy(),
                list(self.config.get('templates', {}).keys()),
                self,
                groups=list(self.config.get('groups', {}).keys())
            )
            # Replace stack contents with the new editor
            if hasattr(self, 'sequence_stack'):
                try:
                    while self.sequence_stack.count() > 0:
                        w = self.sequence_stack.widget(0)
                        self.sequence_stack.removeWidget(w)
                        w.deleteLater()
                except Exception:
                    pass
                self.sequence_stack.addWidget(new_editor)
                self.sequence_stack.setCurrentWidget(new_editor)
                self.sequence_editor_widget = new_editor
                self.current_sequence_widget = new_editor
                self.current_sequence_name = sequence_name

            # Re-enable autosave
            try:
                self._suppress_auto_save = False
            except Exception:
                pass
            # Clear rebuilding flag
            try:
                self._rebuilding_sequence_editor = False
            except Exception:
                pass
    
    def on_loop_toggled(self, checked):
        """Handle loop checkbox state change."""
        if hasattr(self, 'loop_count_spin'):
            self.loop_count_spin.setEnabled(checked)
    
    def save_config(self):
        """Save configuration to file (unified and robust)."""
        try:
            # Skip saving during initial load to avoid wiping config.json
            if getattr(self, '_suppress_auto_save', False):
                return True
            # Update config from UI before saving
            self.update_config_from_ui()

            # Include search region in config if valid
            if (hasattr(self, 'search_region') and 
                self.search_region is not None and 
                isinstance(self.search_region, (list, tuple)) and 
                len(self.search_region) == 4):
                self.config['search_region'] = list(self.search_region)
            else:
                self.config.pop('search_region', None)

            # Ensure templates directory exists
            os.makedirs(getattr(self, 'images_dir', 'images'), exist_ok=True)

            # Normalize template paths to relative where possible
            if 'templates' in self.config:
                updated_templates = {}
                for name, path in self.config['templates'].items():
                    if not path:
                        continue
                    abs_path = path if os.path.isabs(path) else os.path.join(os.path.dirname(self.config_path), path)
                    abs_path = os.path.normpath(abs_path)
                    if os.path.exists(abs_path):
                        try:
                            rel_path = os.path.relpath(abs_path, os.path.dirname(self.config_path))
                            if not rel_path.startswith('..' + os.sep) or rel_path.startswith('.' + os.sep):
                                updated_templates[name] = rel_path
                            else:
                                updated_templates[name] = abs_path
                        except ValueError:
                            updated_templates[name] = abs_path
                    else:
                        logger.warning(f"Template file not found: {abs_path}")
                        updated_templates[name] = path
                self.config['templates'] = updated_templates

            # Ensure the config directory exists
            config_dir = os.path.dirname(self.config_path)
            if config_dir and not os.path.exists(config_dir):
                os.makedirs(config_dir)

            # Save pretty JSON
            with open(self.config_path, 'w') as f:
                json.dump(self.config, f, indent=4, ensure_ascii=False)

            self.statusBar().showMessage(f"Configuration saved to {os.path.basename(self.config_path)}")
            return True
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save configuration: {str(e)}")
            return False
    
    def save_current_sequence(self, sequence_name=None):
        """Save the current sequence data including loop settings.
        
        Args:
            sequence_name: Optional name of the sequence to save. If None, uses the currently selected sequence.
        """
        if not hasattr(self, 'sequence_editor_widget') or not hasattr(self, 'sequences_list'):
            return False
            
        if not sequence_name:
            current_item = self.sequences_list.currentItem()
            if not current_item:
                return False
            sequence_name = current_item.text()
        
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == sequence_name), None)
        
        if sequence and hasattr(self, 'sequence_editor_widget'):
            try:
                # Only update if this sequence is currently being edited
                if hasattr(self, 'current_sequence_name') and self.current_sequence_name == sequence_name:
                    # Update sequence data from editor
                    sequence_data = self.sequence_editor_widget.get_sequence_data()
                    sequence.update(sequence_data)
                    
                    # Update loop settings
                    if hasattr(self, 'loop_checkbox'):
                        sequence['loop'] = self.loop_checkbox.isChecked()
                        if hasattr(self, 'loop_count_spin'):
                            sequence['loop_count'] = self.loop_count_spin.value()
                
                # Save the config
                return self.save_config()
            except Exception as e:
                logger.error(f"Error saving sequence: {str(e)}")
                QMessageBox.critical(self, "Error", f"Failed to save sequence: {str(e)}")
                return False
        
        # Show input dialog
        print("DEBUG: Showing input dialog")  # Debug print
        name, ok = QInputDialog.getText(
            self, 
            "New Sequence", 
            "Enter sequence name:",
            QLineEdit.EchoMode.Normal,
            default_name
        )
        print(f"DEBUG: Got input - ok: {ok}, name: {name}")  # Debug print
        
        if not ok:
            print("DEBUG: User cancelled")  # Debug print
            return
            
        name = name.strip()
        if not name:
            print("DEBUG: Empty name provided")  # Debug print
            QMessageBox.warning(self, "Error", "Sequence name cannot be empty")
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == sequence_name), None)
        
        if sequence:
            # Update loop controls state
            if hasattr(self, 'loop_checkbox'):
                loop_enabled = sequence.get('loop', False)
                self.loop_checkbox.setChecked(loop_enabled)
                if hasattr(self, 'loop_count_spin'):
                    self.loop_count_spin.setValue(sequence.get('loop_count', 1))
                    self.loop_count_spin.setEnabled(loop_enabled)
            
            # Check if we already have an editor widget
            if not hasattr(self, 'sequence_editor_widget'):
                # Create new sequence editor widget if it doesn't exist
                self.sequence_editor_widget = SequenceEditor(
                    sequence,
                    list(self.config.get('templates', {}).keys()),
                    self,
                    groups=list(self.config.get('groups', {}).keys())
                )
                # Add to stack if not already there
                if self.sequence_stack.indexOf(self.sequence_editor_widget) == -1:
                    self.sequence_stack.addWidget(self.sequence_editor_widget)
            else:
                # Update existing widget with new sequence data
                self.sequence_editor_widget.sequence_data = sequence.copy()
                # Update the name in the UI
                if hasattr(self.sequence_editor_widget, 'name_edit'):
                    self.sequence_editor_widget.name_edit.setText(sequence.get('name', 'New Sequence'))
                # Reload steps
                if hasattr(self.sequence_editor_widget, '_add_steps_from_data'):
                    self.sequence_editor_widget._add_steps_from_data()
            
            # Clear any other widgets from the stack
            for i in range(self.sequence_stack.count()):
                widget = self.sequence_stack.widget(i)
                if widget != self.sequence_editor_widget:
                    self.sequence_stack.removeWidget(widget)
                    if widget and widget.isWidgetType() and widget.parent() is None:
                        widget.deleteLater()
            
            # Show the sequence editor
            self.sequence_stack.setCurrentWidget(self.sequence_editor_widget)
            self.current_sequence_widget = self.sequence_editor_widget
            self.current_sequence_name = sequence_name
            
            # Make sure we're on the sequences tab
            self.tab_widget.setCurrentWidget(self.sequences_tab)
    
    def run_sequence(self):
        """Run the selected sequence."""
        # Check if a sequence is already running
        if hasattr(self, 'worker') and self.worker and self.worker.isRunning():
            QMessageBox.warning(self, "Error", "A sequence is already running")
            return
            
        if not hasattr(self, 'sequence_editor_widget'):
            QMessageBox.warning(self, "Error", "No sequence selected")
            return
            
        # Save current sequence
        if not self.save_current_sequence():
            return
            
        # Get the selected sequence
        current_item = self.sequences_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "Error", "No sequence selected")
            return
            
        sequence_name = current_item.text()
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == sequence_name), None)
        
        if not sequence:
            QMessageBox.warning(self, "Error", f"Sequence '{sequence_name}' not found")
            return
            
        # Ensure the sequence has steps and they have actions
        if not sequence.get('steps') or not isinstance(sequence['steps'], list):
            QMessageBox.warning(self, "Error", f"Sequence '{sequence_name}' has no steps")
            return
            
        # Validate templates and steps
        available_templates = set(self.config.get('templates', {}).keys())
        for i, step in enumerate(sequence['steps'], 1):
            # Skip validation for steps without a template (like MOVE actions)
            template_name = step.get('find', '')
            if not template_name or str(template_name).lower() == 'none':
                continue
                
            if template_name not in available_templates:
                QMessageBox.warning(
                    self, 
                    "Template Not Found", 
                    f"Template '{template_name}' not found in step {i}.\n"
                    f"Available templates: {', '.join(available_templates) or 'None'}"
                )
                return
                
            if not step.get('actions') or not isinstance(step['actions'], list):
                QMessageBox.warning(self, "Error", f"Step {i} in sequence '{sequence_name}' has no actions")
                return

        # Proactively ensure all referenced templates are loaded in the bot
        try:
            needed_names = set(
                step.get('find')
                for step in sequence.get('steps', [])
                if isinstance(step, dict) and step.get('find') and str(step.get('find')).lower() != 'none'
            )
            loaded_names = set(getattr(self.bot, 'templates', {}).keys()) if hasattr(self, 'bot') else set()
            missing = list(needed_names - loaded_names)
            if missing and hasattr(self, 'bot'):
                logger.info(f"Loading {len(missing)} missing templates before run: {missing}")
                for name in missing:
                    path = self.config.get('templates', {}).get(name)
                    if not path:
                        continue
                    # Resolve to absolute path similar to load_config
                    abs_path = path if os.path.isabs(path) else os.path.normpath(os.path.join(os.path.dirname(self.config_path), path))
                    if not os.path.exists(abs_path):
                        alt1 = os.path.join(self.script_dir, path)
                        alt2 = os.path.join(self.images_dir, os.path.basename(path))
                        for p in (abs_path, alt1, alt2):
                            if os.path.exists(p):
                                abs_path = p
                                break
                    if os.path.exists(abs_path):
                        try:
                            self.bot.load_template(name, abs_path)
                        except Exception as e:
                            logger.error(f"Failed to load template '{name}' at run start: {e}")
                    else:
                        logger.warning(f"Template file for '{name}' not found before run: {path}")
        except Exception as e:
            logger.debug(f"Pre-run template load skipped due to error: {e}")
        
        # --- Ensure failsafe is only present if enabled ---
        if hasattr(self, 'sequence_editor_widget') and hasattr(self.sequence_editor_widget, 'failsafe_enable_checkbox'):
            if not self.sequence_editor_widget.failsafe_enable_checkbox.isChecked() and 'failsafe' in sequence:
                sequence.pop('failsafe', None)
        
        # Get loop settings from UI controls to avoid config overwrites
        should_loop = self.loop_checkbox.isChecked() if hasattr(self, 'loop_checkbox') else sequence.get('loop', False)
        loop_count = self.loop_count_spin.value() if hasattr(self, 'loop_count_spin') else sequence.get('loop_count', 1)
        
        # Compute max runtime from Break Settings UI for status and worker
        max_secs = None
        hrs = mins = secs_part = 0
        try:
            if hasattr(self, 'break_enabled_checkbox') and self.break_enabled_checkbox.isChecked():
                hrs = self.break_hours_spin.value() if hasattr(self, 'break_hours_spin') else 0
                mins = self.break_minutes_spin.value() if hasattr(self, 'break_minutes_spin') else 0
                secs_part = self.break_seconds_spin.value() if hasattr(self, 'break_seconds_spin') else 0
                max_secs = int(hrs) * 3600 + int(mins) * 60 + int(secs_part)
                if max_secs <= 0:
                    max_secs = None
        except Exception:
            max_secs = None

        # Keep UI responsive during execution; only enable Stop and update status
        status_text = f"Running sequence: {sequence_name}"
        if should_loop:
            if loop_count > 1:
                status_text += f" (Looping {loop_count} times)"
            else:
                status_text += " (Looping)"
        # Do not include elapsed here; timer will update it live
        if max_secs is not None:
            status_text += f" — Max runtime: {hrs}h {mins}m {secs_part}s"
        self.statusBar().showMessage(status_text)
        # Seed the live status prefix and start the timer
        self.run_status_prefix = status_text
        try:
            # Initialize timing attributes and a 1s UI timer
            import time
            self.run_start_time = time.time()
            self.run_max_secs = max_secs
            if hasattr(self, 'run_timer') and self.run_timer:
                self.run_timer.stop()
            self.run_timer = QTimer(self)
            self.run_timer.setInterval(1000)
            self.run_timer.timeout.connect(self.update_run_status_text)
            self.run_timer.start()
        except Exception:
            pass
        
        # Reset break-hit flag for this run
        self._break_time_hit = False
        # Update bot with search region
        self.bot.search_region = self.search_region
        
        # Clean up any existing worker
        if hasattr(self, 'worker') and self.worker is not None:
            try:
                if self.worker.isRunning():
                    self.worker.stop()
                    if not self.worker.wait(1000):  # Wait up to 1 second
                        self.worker.terminate()
                        self.worker.wait()
                self.worker.deleteLater()
            except Exception as e:
                logger.error(f"Error cleaning up worker: {e}")
            finally:
                self.worker = None
        
        # Create and start worker thread
        failsafe_config = self.get_failsafe_config()
        self.worker = BotWorker(
            bot=self.bot,
            sequence=sequence,
            loop=should_loop,
            loop_count=loop_count,
            non_required_wait=self.non_required_wait_checkbox.isChecked(),
            failsafe_config=failsafe_config if failsafe_config else None,
            max_runtime_seconds=max_secs,
            groups=self.config.get('groups', {})
        )
        logger.info(f"Starting sequence with non_required_wait={self.worker.non_required_wait}")
        self.worker.signals.update.connect(self.on_worker_update)
        self.worker.signals.finished.connect(self.on_worker_finished)
        self.worker.signals.error.connect(self.on_worker_error)
        
        # Enable stop button and start the worker
        if hasattr(self, 'stop_action'):
            self.stop_action.setEnabled(True)
            
        self.worker.start()
    
    def on_worker_update(self, data: dict):
        """Handle worker progress updates."""
        if 'status' in data:
            # Update the live status prefix and refresh the timer-driven text
            self.run_status_prefix = data['status']
            self.update_run_status_text()
            # Detect when max runtime is reached to trigger final sequence later
            try:
                if isinstance(data['status'], str) and 'Max runtime reached' in data['status']:
                    self._break_time_hit = True
            except Exception:
                pass
        if 'progress' in data:
            # Update progress bar if you have one
            pass
            
    def on_worker_finished(self):
        """Handle worker completion."""
        # UI remains enabled; just toggle Stop state
        self.stop_action.setEnabled(False)
        if hasattr(self, 'run_action'):
            self.run_action.setEnabled(True)
        # Stop the live timer and clear timing state
        try:
            if hasattr(self, 'run_timer') and self.run_timer:
                self.run_timer.stop()
            self.run_start_time = None
            self.run_max_secs = None
            self.run_status_prefix = ""
        except Exception:
            pass
        self.statusBar().showMessage("Sequence completed successfully")

        # If a preempted scheduled run is pending, start it now
        try:
            if self._schedule_pending and not (hasattr(self, 'worker') and self.worker and self.worker.isRunning()):
                seq_name = self._schedule_pending.get('sequence_name')
                if seq_name:
                    started = self.start_sequence_by_name(seq_name)
                    self._current_run_is_scheduled = bool(started)
                    # If we should resume previous after scheduled, store snapshot for later
                    if started and self._schedule_pending.get('resume_previous') and self._schedule_pending.get('original'):
                        self._resume_after_scheduled = self._schedule_pending['original']
                    self._schedule_pending = None
                    return  # avoid running final-break sequence immediately after
        except Exception:
            pass

        # If a queued scheduled run exists, start it next
        try:
            if getattr(self, '_scheduled_queue', None) and len(self._scheduled_queue) > 0 and not (hasattr(self, 'worker') and self.worker and self.worker.isRunning()):
                item = self._scheduled_queue.pop(0)
                started = self.start_sequence_by_name(item.get('sequence_name'))
                self._current_run_is_scheduled = bool(started)
                if started:
                    return
        except Exception:
            pass

        # If the just-finished run was a scheduled run and we have a resume snapshot, restart previous
        try:
            if getattr(self, '_current_run_is_scheduled', False) and self._resume_after_scheduled and not (hasattr(self, 'worker') and self.worker and self.worker.isRunning()):
                snap = self._resume_after_scheduled
                seq = snap.get('sequence')
                if seq:
                    failsafe_config = self.get_failsafe_config()
                    self.worker = BotWorker(
                        bot=self.bot,
                        sequence=seq,
                        loop=bool(snap.get('loop', False)),
                        loop_count=int(snap.get('loop_count', 1)),
                        non_required_wait=bool(snap.get('non_required_wait', False)),
                        failsafe_config=failsafe_config if failsafe_config else None,
                        max_runtime_seconds=None,
                        groups=self.config.get('groups', {})
                    )
                    self.worker.signals.update.connect(self.on_worker_update)
                    self.worker.signals.finished.connect(self.on_worker_finished)
                    self.worker.signals.error.connect(self.on_worker_error)
                    if hasattr(self, 'stop_action'):
                        self.stop_action.setEnabled(True)
                    if hasattr(self, 'run_action'):
                        self.run_action.setEnabled(False)
                    self._current_run_is_scheduled = False
                    self._resume_after_scheduled = None
                    self.statusBar().showMessage("Resuming previous run after scheduled")
                    self.worker.start()
                    return
        except Exception:
            pass
        # If break time was hit and user enabled final sequence, run it once
        try:
            br = self.config.get('break_settings', {})
            run_final = bool(br.get('run_final_after_break', False))
            final_name = br.get('final_sequence_name')
            if run_final and getattr(self, '_break_time_hit', False) and final_name and final_name != "(none)":
                seq = next((s for s in self.config.get('sequences', []) if s.get('name') == final_name), None)
                if seq:
                    # Start final sequence without max runtime cap
                    self.statusBar().showMessage(f"Running final sequence: {final_name}")
                    # Enable stop and disable run
                    if hasattr(self, 'stop_action'):
                        self.stop_action.setEnabled(True)
                    if hasattr(self, 'run_action'):
                        self.run_action.setEnabled(False)
                    # Clean old worker if any
                    if hasattr(self, 'worker') and self.worker is not None:
                        try:
                            if self.worker.isRunning():
                                self.worker.stop()
                                self.worker.wait(500)
                            self.worker.deleteLater()
                        except Exception:
                            pass
                        self.worker = None
                    failsafe_config = self.get_failsafe_config()
                    self.worker = BotWorker(
                        bot=self.bot,
                        sequence=seq,
                        loop=seq.get('loop', False),
                        loop_count=seq.get('loop_count', 1),
                        non_required_wait=self.non_required_wait_checkbox.isChecked() if hasattr(self, 'non_required_wait_checkbox') else False,
                        failsafe_config=failsafe_config if failsafe_config else None,
                        max_runtime_seconds=None,
                        groups=self.config.get('groups', {})
                    )
                    self.worker.signals.update.connect(self.on_worker_update)
                    self.worker.signals.finished.connect(self.on_worker_finished)
                    self.worker.signals.error.connect(self.on_worker_error)
                    # Reset flag so it only triggers once
                    self._break_time_hit = False
                    self.worker.start()
        except Exception as e:
            logger.debug(f"Final sequence after break failed to start: {e}")

    def start_sequence_by_name(self, sequence_name: str) -> bool:
        """Start a sequence by its name, using current UI loop and failsafe settings."""
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == sequence_name), None)
        if not sequence:
            self.statusBar().showMessage(f"Sequence '{sequence_name}' not found for scheduled run")
            return False
        # If a worker is already running, skip
        if hasattr(self, 'worker') and self.worker and self.worker.isRunning():
            self.statusBar().showMessage(f"Skipped scheduled run '{sequence_name}': worker already running")
            return False
        # Loop and max runtime: prefer sequence metadata when triggered via IPC; otherwise use UI controls
        if getattr(self, '_ipc_run_request', False):
            should_loop = bool(sequence.get('loop', False))
            try:
                loop_count = int(sequence.get('loop_count', 1))
            except Exception:
                loop_count = 1
        else:
            should_loop = self.loop_checkbox.isChecked() if hasattr(self, 'loop_checkbox') else sequence.get('loop', False)
            loop_count = self.loop_count_spin.value() if hasattr(self, 'loop_count_spin') else sequence.get('loop_count', 1)
        # Clamp to sensible bounds
        try:
            loop_count = max(1, min(int(loop_count or 1), 999999))
        except Exception:
            loop_count = 1
        # Compute max runtime: for IPC/web-start runs, prefer config break_settings; scheduled runs ignore cap
        max_secs = None
        hrs = mins = secs_part = 0
        try:
            if getattr(self, '_ipc_run_request', False):
                br = self.config.get('break_settings', {}) or {}
                if bool(br.get('enabled', False)):
                    try:
                        max_secs = int(br.get('max_runtime_seconds', 0) or 0)
                    except Exception:
                        max_secs = 0
                    if max_secs and max_secs > 0:
                        hrs = max_secs // 3600
                        mins = (max_secs % 3600) // 60
                        secs_part = (max_secs % 60)
                    else:
                        max_secs = None
        except Exception:
            max_secs = None
        # Start worker
        failsafe_config = self.get_failsafe_config()
        self.worker = BotWorker(
            bot=self.bot,
            sequence=sequence,
            loop=should_loop,
            loop_count=loop_count,
            non_required_wait=self.non_required_wait_checkbox.isChecked() if hasattr(self, 'non_required_wait_checkbox') else False,
            failsafe_config=failsafe_config if failsafe_config else None,
            max_runtime_seconds=max_secs,
            groups=self.config.get('groups', {})
        )
        self.worker.signals.update.connect(self.on_worker_update)
        self.worker.signals.finished.connect(self.on_worker_finished)
        self.worker.signals.error.connect(self.on_worker_error)
        if hasattr(self, 'stop_action'):
            self.stop_action.setEnabled(True)
        if hasattr(self, 'run_action'):
            self.run_action.setEnabled(False)
        self.worker.start()
        # Initialize status/timer so break runtime is reflected in status bar for web-start
        try:
            status_text = f"Running sequence: {sequence_name}"
            if should_loop:
                if loop_count > 1:
                    status_text += f" (Looping {loop_count} times)"
                else:
                    status_text += " (Looping)"
            if max_secs is not None:
                status_text += f" — Max runtime: {hrs}h {mins}m {secs_part}s"
            self.statusBar().showMessage(status_text)
            self.run_status_prefix = status_text
            import time
            self.run_start_time = time.time()
            self.run_max_secs = max_secs
            self._break_time_hit = False
            if hasattr(self, 'run_timer') and self.run_timer:
                self.run_timer.stop()
            self.run_timer = QTimer(self)
            self.run_timer.setInterval(1000)
            self.run_timer.timeout.connect(self.update_run_status_text)
            self.run_timer.start()
        except Exception:
            pass
        self.statusBar().showMessage(f"Run started: {sequence_name}")
        return True

    def check_schedules(self):
        """Check scheduled sequences and start them at their configured time once per day."""
        try:
            from datetime import datetime
            now = datetime.now()
            today = now.strftime('%Y-%m-%d')
            # Normalize current time components for matching
            curr_h = now.hour
            curr_m = now.minute
            # Ensure UI config is up to date
            schedules = self.config.get('scheduled_sequences', [])
            # Mirror UI to config if rows exist
            if hasattr(self, 'schedule_rows') and self.schedule_rows:
                self.update_config_from_ui()
                schedules = self.config.get('scheduled_sequences', [])
            for sch in schedules:
                if not sch.get('enabled', False):
                    continue
                # Parse scheduled time supporting 12h (hh:mm AM/PM) and 24h (HH:mm)
                sch_time = str(sch.get('time', '')).strip()
                sh, sm = None, None
                try:
                    from datetime import datetime
                    dt = datetime.strptime(sch_time, '%I:%M %p')
                    sh, sm = dt.hour, dt.minute
                except Exception:
                    try:
                        from datetime import datetime
                        dt = datetime.strptime(sch_time, '%H:%M')
                        sh, sm = dt.hour, dt.minute
                    except Exception:
                        pass
                if sh is None or sm is None:
                    continue
                if not (sh == curr_h and sm == curr_m):
                    continue
                last_date = sch.get('last_run_date')
                if last_date == today:
                    continue  # already ran today
                seq_name = sch.get('sequence_name')
                if not seq_name or seq_name == "(none)":
                    continue
                # Decide behavior based on busy state and toggles
                if hasattr(self, 'worker') and self.worker and self.worker.isRunning():
                    if sch.get('preempt_if_busy', False):
                        # Capture current run to optionally resume later
                        try:
                            original = {
                                'sequence': getattr(self.worker, 'sequence', None),
                                'loop': (self.loop_checkbox.isChecked() if hasattr(self, 'loop_checkbox') else False),
                                'loop_count': (self.loop_count_spin.value() if hasattr(self, 'loop_count_spin') else 1),
                                'non_required_wait': (self.non_required_wait_checkbox.isChecked() if hasattr(self, 'non_required_wait_checkbox') else False),
                            }
                        except Exception:
                            original = None
                        self._schedule_pending = {
                            'sequence_name': seq_name,
                            'resume_previous': bool(sch.get('resume_previous', False)),
                            'original': original,
                        }
                        # Stop current; on_worker_finished will start scheduled
                        try:
                            self.statusBar().showMessage(f"Preempting current run for scheduled: {seq_name}")
                            self.stop_sequence()
                        except Exception:
                            pass
                        # Update last_run_date now to avoid repeated attempts in this minute
                        sch['last_run_date'] = today
                    elif sch.get('queue_if_busy', False):
                        # Queue the scheduled run to start after current finishes
                        self._scheduled_queue.append({'sequence_name': seq_name})
                        sch['last_run_date'] = today
                        self.statusBar().showMessage(f"Queued scheduled run: {seq_name}")
                    else:
                        # Skip when busy
                        self.statusBar().showMessage(f"Skipped scheduled run '{seq_name}': worker busy")
                else:
                    started = self.start_sequence_by_name(seq_name)
                    self._current_run_is_scheduled = bool(started)
                    if started:
                        sch['last_run_date'] = today
                        # Update UI row state too
                        if hasattr(self, 'schedule_rows'):
                            for row in self.schedule_rows:
                                if row._seq_combo.currentText() == seq_name:
                                    row._last_run_date = today
                        # Persist
                        try:
                            self.save_config()
                        except Exception:
                            pass
            # Refresh next indicator after evaluating schedules
            try:
                self.update_scheduler_indicator()
            except Exception:
                pass
        except Exception as e:
            logger.debug(f"check_schedules error: {e}")

    def handle_ipc_commands(self):
        """Check for an ipc_command.json file and execute commands.
        The web_server writes commands here so the GUI can act directly without relying on global shortcuts.
        Supported commands:
        - {"command": "run", "sequence_name": "Name"}
        - {"command": "stop"}
        """
        try:
            from pathlib import Path
            import sys as _sys
            import json as _json
            # Resolve base directory for IPC and config in both source and frozen builds
            try:
                base_dir = Path(_sys.executable).resolve().parent if getattr(_sys, 'frozen', False) else Path(__file__).resolve().parent
            except Exception:
                base_dir = Path(__file__).resolve().parent
            cmd_path = base_dir / 'ipc_command.json'
            if not cmd_path.exists():
                return
            with cmd_path.open('r', encoding='utf-8') as f:
                data = _json.load(f)
            # Remove command file immediately to avoid reprocessing
            try:
                cmd_path.unlink(missing_ok=True)
            except Exception:
                pass
            cmd = (data or {}).get('command')
            if cmd == 'run':
                seq_name = (data or {}).get('sequence_name')
                if seq_name:
                    try:
                        self._ipc_run_request = True
                    except Exception:
                        pass
                    try:
                        self.start_sequence_by_name(seq_name)
                    finally:
                        try:
                            self._ipc_run_request = False
                        except Exception:
                            pass
                    try:
                        # Switch UI to Sequences tab and select the running sequence
                        if hasattr(self, 'tab_widget') and hasattr(self, 'sequences_tab'):
                            self.tab_widget.setCurrentWidget(self.sequences_tab)
                        if hasattr(self, 'sequences_list') and self.sequences_list:
                            for i in range(self.sequences_list.count()):
                                item = self.sequences_list.item(i)
                                if item and item.text() == seq_name:
                                    self.sequences_list.setCurrentRow(i)
                                    break
                        # Bring the window to foreground so status is visible
                        try:
                            self.raise_()
                            self.activateWindow()
                        except Exception:
                            pass
                    except Exception:
                        pass
                else:
                    # Fallback: start current selection via Run button logic
                    self.run_sequence()
                    try:
                        if hasattr(self, 'tab_widget') and hasattr(self, 'sequences_tab'):
                            self.tab_widget.setCurrentWidget(self.sequences_tab)
                    except Exception:
                        pass
                try:
                    # Show a temporary status, then revert to live step/progress updates
                    self.statusBar().showMessage("IPC: run requested", 1500)
                except Exception:
                    pass
            elif cmd == 'stop':
                self.stop_sequence()
                try:
                    self.statusBar().showMessage("IPC: stop requested", 1500)
                except Exception:
                    pass
            elif cmd == 'reload_config':
                try:
                    # Reload configuration from disk and refresh UI
                    from pathlib import Path as _Path
                    try:
                        _base = _Path(_sys.executable).resolve().parent if getattr(_sys, 'frozen', False) else _Path(__file__).resolve().parent
                    except Exception:
                        _base = _Path(__file__).resolve().parent
                    project_cfg = str((_base / 'config.json'))
                    # Prefer project config path to align with web_server writes
                    if os.path.exists(project_cfg):
                        self.load_config(project_cfg)
                    else:
                        self.load_config()
                    self.statusBar().showMessage("IPC: config reloaded", 1500)
                except Exception as e:
                    logger.debug(f"IPC reload_config failed: {e}")
            elif cmd == 'set_non_required_wait':
                try:
                    val = bool((data or {}).get('value', False))
                    if hasattr(self, 'non_required_wait_checkbox') and self.non_required_wait_checkbox:
                        self.non_required_wait_checkbox.setChecked(val)
                        self.statusBar().showMessage("IPC: set non-required-wait", 1500)
                except Exception as e:
                    logger.debug(f"IPC set_non_required_wait failed: {e}")
        except Exception as e:
            logger.debug(f"IPC command handling failed: {e}")

    def update_scheduler_indicator(self):
        """Compute and display the next scheduled run (sequence @ time) in the status bar."""
        try:
            from datetime import datetime, timedelta
            # Mirror current UI schedules to config if rows exist
            try:
                if hasattr(self, 'schedule_rows') and self.schedule_rows:
                    self.update_config_from_ui()
            except Exception:
                pass
            schedules = self.config.get('scheduled_sequences', [])
            now = datetime.now()
            best_dt = None
            best_name = None

            def parse_time_str(s: str):
                s = str(s or '').strip()
                try:
                    from datetime import datetime
                    return datetime.strptime(s, '%I:%M %p').time()
                except Exception:
                    try:
                        from datetime import datetime
                        return datetime.strptime(s, '%H:%M').time()
                    except Exception:
                        return None

            for sch in schedules:
                if not sch.get('enabled', False):
                    continue
                seq_name = sch.get('sequence_name')
                if not seq_name or seq_name == '(none)':
                    continue
                t = parse_time_str(sch.get('time'))
                if t is None:
                    continue
                candidate_today = now.replace(hour=t.hour, minute=t.minute, second=0, microsecond=0)
                candidate = candidate_today if candidate_today >= now else (candidate_today + timedelta(days=1))
                if best_dt is None or candidate < best_dt:
                    best_dt = candidate
                    best_name = seq_name

            if not hasattr(self, 'scheduler_label'):
                return
            if best_dt is None:
                self.scheduler_label.setText('Next: (none)')
            else:
                h24 = best_dt.hour
                m = best_dt.minute
                ap = 'AM' if h24 < 12 else 'PM'
                h12 = h24 % 12
                if h12 == 0:
                    h12 = 12
                hhmm_ap = f"{h12:02d}:{m:02d} {ap}"
                day_hint = '' if best_dt.date() == now.date() else ' (tomorrow)'
                self.scheduler_label.setText(f"Next: {best_name} @ {hhmm_ap}{day_hint}")
        except Exception:
            pass

    def _on_schedule_row_changed(self):
        """Persist schedule UI to config and refresh the next-run indicator."""
        try:
            # Avoid write-backs during initialization
            if not getattr(self, '_suppress_auto_save', False):
                self.update_config_from_ui()
                self.save_config()
        except Exception:
            pass
        try:
            self.update_scheduler_indicator()
        except Exception:
            pass
        
    def on_worker_error(self, error: str):
        """Handle worker errors."""
        # UI remains enabled; toggle actions state
        # Stop the live timer and clear timing state
        try:
            if hasattr(self, 'run_timer') and self.run_timer:
                self.run_timer.stop()
            self.run_start_time = None
            self.run_max_secs = None
            self.run_status_prefix = ""
        except Exception:
            pass
        QMessageBox.critical(self, "Error", error)
        self.statusBar().showMessage(f"Error: {error}")
        if hasattr(self, 'run_action'):
            self.run_action.setEnabled(True)
        
    def stop_sequence(self):
        """Stop the currently running sequence."""
        logger.info("=== STOP SEQUENCE REQUESTED ===")
        
        # Ensure this runs in the main thread
        if not hasattr(self, 'worker') or not self.worker:
            logger.warning("No worker found to stop")
            return False
            
        try:
            # Get a reference to the worker and clear it immediately
            worker = self.worker
            self.worker = None
            
            # Request the worker to stop
            worker.stop()
            
            # Update UI immediately
            self.stop_action.setEnabled(False)
            self.statusBar().showMessage("Stopping sequence...")
            self.setEnabled(True)
            # Stop the live timer and clear timing state
            try:
                if hasattr(self, 'run_timer') and self.run_timer:
                    self.run_timer.stop()
                self.run_start_time = None
                self.run_max_secs = None
                self.run_status_prefix = ""
            except Exception:
                pass
            
            # Force stop if not responding
            def check_worker():
                if worker.isRunning():
                    logger.warning("Worker still running, forcing termination...")
                    worker.terminate()
                    if worker.wait(1000):  # Wait up to 1 second
                        logger.info("Worker terminated successfully")
                    else:
                        logger.error("Failed to terminate worker")
                
                try:
                    worker.deleteLater()
                    logger.info("=== STOP SEQUENCE COMPLETED ===")
                    self.statusBar().showMessage("Sequence stopped")
                except Exception as e:
                    logger.error(f"Error during cleanup: {e}", exc_info=True)
                    self.statusBar().showMessage("Error stopping sequence")
            
            # Check after a short delay
            QTimer.singleShot(100, check_worker)
            return True
            
        except Exception as e:
            logger.error(f"=== ERROR STOPPING SEQUENCE: {e} ===", exc_info=True)
            # Ensure UI is re-enabled even if there was an error
            self.setEnabled(True)
            self.stop_action.setEnabled(False)
            self.statusBar().showMessage("Error stopping sequence")
            return False

    def update_run_status_text(self):
        """Update the status bar with live elapsed and max runtime."""
        try:
            if not hasattr(self, 'run_start_time') or self.run_start_time is None:
                return
            import time
            elapsed = int(time.time() - self.run_start_time)
            def fmt(secs: int) -> str:
                h = max(secs // 3600, 0)
                m = max((secs % 3600) // 60, 0)
                s = max(secs % 60, 0)
                return f"{h}h {m}m {s}s"
            prefix = getattr(self, 'run_status_prefix', '') or 'Running'
            if hasattr(self, 'run_max_secs') and self.run_max_secs is not None:
                msg = f"{prefix} — Elapsed: {fmt(elapsed)} / Max: {fmt(int(self.run_max_secs))}"
            else:
                msg = f"{prefix} — Elapsed: {fmt(elapsed)}"
            self.statusBar().showMessage(msg)
        except Exception:
            pass
    
    def toggle_failsafe_ui(self, enabled: bool):
        """Enable/disable failsafe UI elements."""
        try:
            logger.info(f"toggle_failsafe_ui: enabled={enabled}")
        except Exception:
            pass
        # Reflect enabled flag into in-memory config so subsequent UI refreshes don't immediately flip it back
        try:
            fs = self.config.get('failsafe', {}) or {}
            fs['enabled'] = bool(enabled)
            # Ensure sequence exists when enabling
            if enabled and not isinstance(fs.get('sequence'), list):
                fs['sequence'] = []
            self.config['failsafe'] = fs
        except Exception:
            pass
        self.failsafe_template_combo.setEnabled(enabled)
        self.failsafe_conf_spin.setEnabled(enabled)
        self.failsafe_region_btn.setEnabled(enabled)
        self.test_failsafe_btn.setEnabled(enabled)
        if hasattr(self, 'failsafe_show_region_btn'):
            self.failsafe_show_region_btn.setEnabled(enabled)
        # Enable/disable new capture/select and name/preview controls
        if hasattr(self, 'fs_capture_btn'):
            self.fs_capture_btn.setEnabled(enabled)
        if hasattr(self, 'fs_select_btn'):
            self.fs_select_btn.setEnabled(enabled)
        if hasattr(self, 'failsafe_template_name_edit'):
            self.failsafe_template_name_edit.setEnabled(enabled)
        # Refresh templates when enabling
        if enabled:
            self.refresh_failsafe_templates()

        if enabled:
            # Switch to failsafe sequence editor
            failsafe_cfg = self.config.get('failsafe', {}) or {}
            steps = failsafe_cfg.get('sequence', []) if isinstance(failsafe_cfg, dict) else []
            # Collect available groups names
            try:
                groups_list = list(self.config.get('groups', {}).keys())
            except Exception:
                groups_list = []
            if not hasattr(self, 'current_failsafe_editor'):
                self.current_failsafe_editor = FailsafeSequenceEditor(
                    {"steps": steps},
                    list(self.config.get("templates", {}).keys()),
                    self,
                    groups_list
                )
                self.failsafe_sequence_stack.addWidget(self.current_failsafe_editor)
            else:
                # Refresh existing editor with current steps
                try:
                    self.current_failsafe_editor.load_sequence(steps)
                    # Update groups list if changed
                    self.current_failsafe_editor.groups = groups_list
                except Exception:
                    pass
            self.failsafe_sequence_stack.setCurrentWidget(self.current_failsafe_editor)
        else:
            # Switch to welcome screen
            self.failsafe_sequence_stack.setCurrentIndex(0)
        # Persist enabled flag to config so subsequent refreshes keep state
        try:
            self.save_config()
        except Exception:
            pass
    
    def select_failsafe_region(self):
        """Select region for failsafe template detection."""
        dialog = ScreenCaptureDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            rect = dialog.get_capture_rect()
            if rect and rect.isValid():
                self.failsafe_region = (rect.x(), rect.y(), rect.width(), rect.height())
                self.failsafe_region_label.setText(f"Region: ({rect.x()}, {rect.y()}, {rect.width()}x{rect.height()})")
            else:
                self.failsafe_region = None
                self.failsafe_region_label.setText("Region: Full Screen")

    def show_failsafe_region_overlay(self):
        """Preview the current failsafe search region on screen."""
        try:
            rect = None
            if hasattr(self, 'failsafe_region') and self.failsafe_region:
                try:
                    x, y, w, h = self.failsafe_region
                    rect = QRect(int(x), int(y), int(w), int(h))
                except Exception:
                    rect = None
            # No per-monitor selection for failsafe; cover full desktop
            dlg = OverlayPreviewWindow(self, rect=rect, point=None, target_geometry=None, duration_ms=1200, message="Failsafe Region")
            try:
                self._overlay_preview = dlg
            except Exception:
                pass
            dlg.show()
        except Exception:
            pass
    
    def test_failsafe_sequence(self):
        """Test the failsafe sequence execution."""
        if not self.failsafe_enable_checkbox.isChecked():
            QMessageBox.warning(self, "Warning", "Failsafe is not enabled")
            return
        
        # Build current failsafe configuration
        failsafe_config = self.get_failsafe_config()
        if not failsafe_config.get('template'):
            QMessageBox.warning(self, "Warning", "Please select a failsafe template")
            return
        # Ensure there is a failsafe sequence to test
        fs_steps = failsafe_config.get('sequence', [])
        if not fs_steps:
            QMessageBox.warning(self, "Warning", "No failsafe sequence configured to test")
            return

        # Keep UI responsive during failsafe test; show status only
        self.statusBar().showMessage("Testing failsafe sequence...")

        # Clean up any existing worker
        if hasattr(self, 'worker') and self.worker is not None:
            try:
                if self.worker.isRunning():
                    self.worker.stop()
                    if not self.worker.wait(1000):  # Wait up to 1 second
                        self.worker.terminate()
                        self.worker.wait()
                self.worker.deleteLater()
            except Exception as e:
                logger.error(f"Error cleaning up worker: {e}")
            finally:
                self.worker = None

        # Create and start a worker to run only the failsafe sequence
        self.worker = BotWorker(
            bot=self.bot,
            sequence={"name": "Failsafe Test", "steps": []},
            loop=False,
            loop_count=1,
            non_required_wait=False,
            failsafe_config=failsafe_config,
            failsafe_only=True,
            groups=self.config.get('groups', {})
        )
        self.worker.signals.update.connect(self.on_worker_update)
        self.worker.signals.finished.connect(self.on_worker_finished)
        self.worker.signals.error.connect(self.on_worker_error)

        if hasattr(self, 'stop_action'):
            self.stop_action.setEnabled(True)

        self.worker.start()
    
    def get_failsafe_config(self) -> dict:
        """Get the current failsafe configuration."""
        if not self.failsafe_enable_checkbox.isChecked():
            return {}
        
        config = {
            "enabled": True,
            "template": self.failsafe_template_combo.currentText(),
            "confidence": self.failsafe_conf_spin.value(),
            "region": getattr(self, 'failsafe_region', None)
        }
        
        # Add failsafe sequence if editor exists
        if hasattr(self, 'current_failsafe_editor'):
            seq_data = self.current_failsafe_editor.get_failsafe_data()
            # Store as a plain list of step dicts for worker consumption
            config["sequence"] = seq_data.get("steps", []) if isinstance(seq_data, dict) else []
        
        return config

    def refresh_failsafe_templates(self):
        """Populate the failsafe template combo from current config templates."""
        try:
            # Preserve the intended selection from config (template or template_name)
            desired = None
            try:
                fs = self.config.get('failsafe', {}) or {}
                desired = fs.get('template') or fs.get('template_name') or None
            except Exception:
                desired = None
            if not desired and hasattr(self, 'failsafe_template_combo'):
                desired = self.failsafe_template_combo.currentText().strip()
            self.failsafe_template_combo.blockSignals(True)
            self.failsafe_template_combo.clear()
            for name in sorted(self.config.get('templates', {}).keys()):
                self.failsafe_template_combo.addItem(name)
            self.failsafe_template_combo.blockSignals(False)
            # Restore selection if available
            if desired:
                idx = self.failsafe_template_combo.findText(desired)
                if idx >= 0:
                    self.failsafe_template_combo.setCurrentIndex(idx)
            # If a selection exists, update preview
            self.update_failsafe_preview()
            # Update templates list for failsafe step editors
            try:
                if hasattr(self, 'current_failsafe_editor'):
                    self.current_failsafe_editor.update_templates(list(self.config.get('templates', {}).keys()))
            except Exception:
                pass
        except Exception:
            # Ensure signals unblocked on error
            self.failsafe_template_combo.blockSignals(False)

    def update_failsafe_preview(self):
        """Update preview image for the selected failsafe template."""
        try:
            name = self.failsafe_template_combo.currentText().strip()
            if hasattr(self, 'failsafe_template_name_edit') and name:
                self.failsafe_template_name_edit.setText(name)
            path = self.config.get('templates', {}).get(name)
            if not path:
                self.failsafe_preview.setText("No preview")
                return
            # Resolve to absolute path relative to config directory
            abs_path = path if os.path.isabs(path) else os.path.join(os.path.dirname(self.config_path), path)
            if os.path.exists(abs_path):
                pixmap = QPixmap(abs_path)
                if not pixmap.isNull():
                    self.failsafe_preview.setPixmap(
                        pixmap.scaled(self.failsafe_preview.size(),
                                      Qt.AspectRatioMode.KeepAspectRatio,
                                      Qt.TransformationMode.SmoothTransformation))
                    return
            self.failsafe_preview.setText("No preview")
        except Exception:
            self.failsafe_preview.setText("No preview")

    def capture_failsafe_image(self):
        """Capture a screen region and save as a failsafe template, updating config."""
        try:
            name = self.failsafe_template_name_edit.text().strip() if hasattr(self, 'failsafe_template_name_edit') else ''
            if not name:
                name = f"failsafe_{int(datetime.now().timestamp())}"
                if hasattr(self, 'failsafe_template_name_edit'):
                    self.failsafe_template_name_edit.setText(name)

            capture_dialog = ScreenCaptureDialog(self)
            if capture_dialog.exec() != QDialog.DialogCode.Accepted:
                return False
            rect = capture_dialog.get_capture_rect()
            if not rect or not rect.isValid():
                return False

            # Capture region
            screenshot = ImageGrab.grab(bbox=(
                rect.x(),
                rect.y(),
                rect.x() + rect.width(),
                rect.y() + rect.height()
            ))

            # Ensure destination directory
            os.makedirs(self.failsafe_images_dir, exist_ok=True)
            dest_path = os.path.join(self.failsafe_images_dir, f"{name}.png")
            # If exists, confirm overwrite
            if os.path.exists(dest_path):
                reply = QMessageBox.question(
                    self, 'File Exists',
                    f"An image named '{name}.png' already exists. Overwrite?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No
                )
                if reply != QMessageBox.StandardButton.Yes:
                    return False
            screenshot.save(dest_path, "PNG")

            # Store relative path in config
            rel_path = os.path.relpath(dest_path, os.path.dirname(self.config_path))
            self.config.setdefault('templates', {})[name] = rel_path

            # Load into bot
            abs_path = os.path.abspath(os.path.join(os.path.dirname(self.config_path), rel_path))
            if hasattr(self, 'bot') and os.path.exists(abs_path):
                self.bot.load_template(name, abs_path)

            # Refresh UI lists
            self.refresh_failsafe_templates()
            # Select the new template
            idx = self.failsafe_template_combo.findText(name)
            if idx >= 0:
                self.failsafe_template_combo.setCurrentIndex(idx)
            self.update_template_tester_templates()

            # Save config
            self.save_config()
            self.statusBar().showMessage(f"Captured failsafe image '{name}'")
            return True
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to capture failsafe image: {str(e)}")
            return False

    def select_failsafe_image(self):
        """Select an image file and add as a failsafe template."""
        try:
            file_path, _ = QFileDialog.getOpenFileName(
                self, "Select Failsafe Image", "",
                "Images (*.png *.jpg *.jpeg *.bmp)"
            )
            if not file_path:
                return False
            base_name = os.path.splitext(os.path.basename(file_path))[0]
            name = self.failsafe_template_name_edit.text().strip() if hasattr(self, 'failsafe_template_name_edit') else ''
            if not name:
                name = base_name
                if hasattr(self, 'failsafe_template_name_edit'):
                    self.failsafe_template_name_edit.setText(name)

            # Ensure destination directory
            os.makedirs(self.failsafe_images_dir, exist_ok=True)
            ext = os.path.splitext(file_path)[1]
            dest_path = os.path.join(self.failsafe_images_dir, f"{name}{ext}")

            try:
                if os.path.exists(dest_path):
                    # Check if same file; if different, ask overwrite
                    try:
                        same = os.path.samefile(file_path, dest_path)
                    except Exception:
                        same = False
                    if not same:
                        reply = QMessageBox.question(
                            self, 'File Exists',
                            f"An image named '{name}{ext}' already exists. Overwrite?",
                            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                            QMessageBox.StandardButton.No
                        )
                        if reply != QMessageBox.StandardButton.Yes:
                            return False
                shutil.copy2(file_path, dest_path)
            except shutil.SameFileError:
                # Use existing file
                pass

            # Store relative path and load into bot
            rel_path = os.path.relpath(dest_path, os.path.dirname(self.config_path))
            self.config.setdefault('templates', {})[name] = rel_path
            abs_path = os.path.abspath(os.path.join(os.path.dirname(self.config_path), rel_path))
            if hasattr(self, 'bot') and os.path.exists(abs_path):
                self.bot.load_template(name, abs_path)

            # Refresh and select
            self.refresh_failsafe_templates()
            idx = self.failsafe_template_combo.findText(name)
            if idx >= 0:
                self.failsafe_template_combo.setCurrentIndex(idx)
            self.update_template_tester_templates()

            # Save config
            self.save_config()
            self.statusBar().showMessage(f"Loaded failsafe image '{name}'")
            return True
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load failsafe image: {str(e)}")
            return False
    def show_about(self):
        """Show about dialog."""
        QMessageBox.about(
            self,
            "About Image Detection Bot",
            "<h2>Image Detection Bot</h2>"
            "<p>Version 1.0.0</p>"
            "<p>A tool for automating tasks based on image detection.</p>"
            "<p>Created with Python and PyQt6</p>"
        )
    
    def update_mouse_position(self):
        """Update the mouse position in the status bar."""
        try:
            pos = pyautogui.position()
            self.mouse_pos_label.setText(f"X: {pos.x}, Y: {pos.y}")
        except Exception as e:
            logger.error(f"Error updating mouse position: {e}")
    
    def keyPressEvent(self, event):
        """Handle key press events."""
        # Check for F8 key press
        if event.key() == Qt.Key.Key_F8:
            self.f8_pressed = True
            if self.stop_sequence():
                event.accept()
                return
        super().keyPressEvent(event)
        
    def keyReleaseEvent(self, event):
        """Handle key release events."""
        if event.key() == Qt.Key.Key_F8:
            self.f8_pressed = False
        super().keyReleaseEvent(event)
        
    def check_f8_key(self):
        """Check if F8 key is pressed using direct keyboard state."""
        try:
            import ctypes
            VK_F8 = 0x77  # Virtual key code for F8
            key_state = ctypes.windll.user32.GetAsyncKeyState(VK_F8)
            is_pressed = key_state & 0x8000 != 0
            
            if is_pressed and not self.f8_pressed:
                self.f8_pressed = True
                self.stop_sequence()
            elif not is_pressed and self.f8_pressed:
                self.f8_pressed = False
        except Exception as e:
            logger.error(f"Error checking F8 key: {e}")
    
    def closeEvent(self, event):
        """Handle window close event."""
        # Stop the mouse position timer
        if hasattr(self, 'mouse_timer') and self.mouse_timer.isActive():
            self.mouse_timer.stop()
            
        # Stop the F8 key check timer
        if hasattr(self, 'f8_timer') and self.f8_timer.isActive():
            self.f8_timer.stop()
        
        # Stop any running worker threads
        if hasattr(self, 'worker') and self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait()
        
        # Clean up the F8 shortcut if it exists
        if hasattr(self, 'f8_shortcut'):
            try:
                self.f8_shortcut.activated.disconnect()
                self.f8_shortcut.deleteLater()
            except RuntimeError:
                pass  # Already deleted or disconnected
        
        # Ask to save changes if needed
        # TODO: Check for unsaved changes
        
        event.accept()

def apply_dark_theme(app):
    """Apply a dark theme to the application."""
    # Use dark theme if system is in dark mode
    if darkdetect.isDark():
        app.setStyle("Fusion")
        
        dark_palette = app.palette()
        dark_palette.setColor(dark_palette.ColorRole.Window, Qt.GlobalColor.darkGray)
        dark_palette.setColor(dark_palette.ColorRole.WindowText, Qt.GlobalColor.white)
        dark_palette.setColor(dark_palette.ColorRole.Base, Qt.GlobalColor.darkGray)
        dark_palette.setColor(dark_palette.ColorRole.AlternateBase, Qt.GlobalColor.gray)
        dark_palette.setColor(dark_palette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
        dark_palette.setColor(dark_palette.ColorRole.ToolTipText, Qt.GlobalColor.white)
        dark_palette.setColor(dark_palette.ColorRole.Text, Qt.GlobalColor.white)
        dark_palette.setColor(dark_palette.ColorRole.Button, Qt.GlobalColor.darkGray)
        dark_palette.setColor(dark_palette.ColorRole.ButtonText, Qt.GlobalColor.white)
        dark_palette.setColor(dark_palette.ColorRole.BrightText, Qt.GlobalColor.red)
        dark_palette.setColor(dark_palette.ColorRole.Link, Qt.GlobalColor.cyan)
        dark_palette.setColor(dark_palette.ColorRole.Highlight, Qt.GlobalColor.blue)
        dark_palette.setColor(dark_palette.ColorRole.HighlightedText, Qt.GlobalColor.white)
        
        app.setPalette(dark_palette)
        app.setStyleSheet("""
            QToolTip {
                color: #ffffff;
                background-color: #2a82da;
                border: 1px solid white;
            }
            QGroupBox {
                border: 1px solid gray;
                border-radius: 3px;
                margin-top: 0.5em;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }
        """)

def main():
    # Create application
    app = QApplication(sys.argv)
    
    # Apply dark theme if needed
    apply_dark_theme(app)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    # Run application
    sys.exit(app.exec())

if __name__ == "__main__":
    main()