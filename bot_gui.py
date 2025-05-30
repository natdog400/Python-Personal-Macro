import os
import sys
import json
import logging
import tempfile
import shutil
from datetime import datetime
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QListWidget, QStackedWidget, QLineEdit, QSpinBox, 
                            QDoubleSpinBox, QComboBox, QFileDialog, QMessageBox, QCheckBox, 
                            QGroupBox, QScrollArea, QSplitter, QFrame, QSizePolicy, QToolBar, 
                            QStatusBar, QDialog, QFormLayout, QListWidgetItem, QInputDialog, QMenu,
                            QDialogButtonBox, QMenuBar, QTableWidget, QTableWidgetItem,
                            QHeaderView, QAbstractItemView, QTabWidget)
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
    def __init__(self, bot, sequence, loop=False, loop_count=1):
        super().__init__()
        self.bot = bot
        self.sequence = sequence
        self.loop = loop
        self.loop_count = loop_count if loop else 1
        self.signals = WorkerSignals()
        self._is_running = True
        self._should_stop = False  # Flag to indicate if we should stop
        self.current_step = 0
        self.total_steps = 0
        self.current_iteration = 0
        self._process_pid = None  # Store the process ID for forceful termination
        logger.info("BotWorker initialized with loop=%s, loop_count=%s", loop, loop_count)
    
    def run(self):
        """Run the sequence of steps with optional looping."""
        # Store the process ID for forceful termination if needed
        self._process_pid = os.getpid()
        logger.info(f"BotWorker started with PID: {self._process_pid}")
        
        # --- Failsafe config ---
        failsafe = self.sequence.get('failsafe', None)
        failsafe_template = None
        failsafe_goto_step = None
        failsafe_confidence = 0.9
        failsafe_region = None
        if failsafe:
            failsafe_template = failsafe.get('template')
            failsafe_goto_step = failsafe.get('goto_step')
            failsafe_confidence = failsafe.get('confidence', 0.9)
            failsafe_region = failsafe.get('region')
        
        try:
            while self._is_running and not self._should_stop and (not self.loop or self.current_iteration < self.loop_count):
                self.current_iteration += 1
                logger.info("Starting iteration %d/%d", self.current_iteration, self.loop_count if self.loop else 1)
                
                # Check for stop request before starting iteration
                if self._should_stop:
                    logger.info("Stop requested before starting iteration")
                    break
                self.total_steps = len(self.sequence.get('steps', []))
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
                    for i, step in enumerate(self.sequence.get('steps', [])):
                        if not self._is_running:
                            should_continue = False
                            break
                        
                        self.current_step = i
                        template_name = step.get('find', '')
                        required = step.get('required', True)
                        timeout = step.get('timeout', 10.0)
                        confidence = step.get('confidence', 0.9)
                        search_region = step.get('search_region')
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
                                        if failsafe_template is not None and failsafe_goto_step is not None:
                                            pos = self.bot.find_image(
                                                template_name=failsafe_template,
                                                region=failsafe_region,
                                                timeout=0.5,
                                                confidence=failsafe_confidence
                                            )
                                            if pos is not None:
                                                self.signals.update.emit({
                                                    "status": f"Failsafe '{failsafe_template}' detected! Jumping to step {failsafe_goto_step+1}.",
                                                    "progress": int((i / self.total_steps) * 100)
                                                })
                                                i = failsafe_goto_step
                                                break  # break out of actions, will continue at new step
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
                                        position = self.bot.find_image(
                                            template_name=template_name,
                                            region=search_region,
                                            timeout=timeout if required else 0.5,
                                            confidence=confidence
                                        )
                                    else:
                                        start_time = datetime.now()
                                        check_interval = 0.1
                                        while not self._should_stop and (datetime.now() - start_time).total_seconds() < (timeout if required else 0.5):
                                            position = self.bot.find_image(
                                                template_name=template_name,
                                                confidence=confidence
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
                                            if failsafe_template is not None and failsafe_goto_step is not None:
                                                pos = self.bot.find_image(
                                                    template_name=failsafe_template,
                                                    region=failsafe_region,
                                                    timeout=0.5,
                                                    confidence=failsafe_confidence
                                                )
                                                if pos is not None:
                                                    self.signals.update.emit({
                                                        "status": f"Failsafe '{failsafe_template}' detected! Jumping to step {failsafe_goto_step+1}.",
                                                        "progress": int((i / self.total_steps) * 100)
                                                    })
                                                    i = failsafe_goto_step
                                                    break  # break out of actions, will continue at new step
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
    """Dialog for capturing a region of the screen."""
    def __init__(self, parent=None):
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
        
        # Calculate combined geometry of all screens
        self.combined_geometry = QRect()
        for screen in self.screens:
            self.combined_geometry = self.combined_geometry.united(screen.geometry())
        
        # Set dialog to cover all screens
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
            
            # Draw overlay for all screens
            for screen in QGuiApplication.screens():
                screen_geom = screen.geometry()
                painter.drawRect(screen_geom)
            
            # Draw the selection rectangle if we have valid points
            if (self.start_pos and self.end_pos and 
                not self.start_pos.isNull() and not self.end_pos.isNull() and
                self.selection_rect and self.selection_rect.isValid()):
                
                # Draw the selection border
                painter.setPen(QPen(Qt.GlobalColor.red, 2, Qt.PenStyle.SolidLine))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                
                # Convert to local coordinates if needed
                local_rect = QRect(
                    self.selection_rect.topLeft() - self.combined_geometry.topLeft(),
                    self.selection_rect.size()
                )
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
            # Ensure the rectangle is within screen bounds
            screen_geom = self.current_screen.geometry()
            rect = self.selection_rect.intersected(screen_geom)
            
            # Convert to screen coordinates if needed
            if rect.isValid() and rect.width() > 5 and rect.height() > 5:  # Minimum size
                return rect
        return None


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
            capture_dialog = ScreenCaptureDialog(self)
            if capture_dialog.exec() != QDialog.DialogCode.Accepted:
                return False
                
            rect = capture_dialog.get_capture_rect()
            if not rect or not rect.isValid():
                return False
                
            try:
                # Capture the screen region
                screenshot = ImageGrab.grab(bbox=(
                    rect.x(), 
                    rect.y(), 
                    rect.x() + rect.width(), 
                    rect.y() + rect.height()
                ))
                
                # Create a temporary file for the screenshot
                temp_dir = os.path.join(self.images_dir, "temp")
                os.makedirs(temp_dir, exist_ok=True)
                temp_filename = f"temp_{int(datetime.now().timestamp())}.png"
                temp_path = os.path.join(temp_dir, temp_filename)
                
                # Save the screenshot
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
        self.params_layout = QHBoxLayout(self.params_widget)
        self.params_layout.setContentsMargins(0, 0, 0, 0)
        
        # Remove button
        remove_btn = QPushButton("Remove")
        remove_btn.clicked.connect(self.remove_self)
        
        layout.addWidget(QLabel("Action:"))
        layout.addWidget(self.type_combo)
        layout.addWidget(self.params_widget)
        layout.addStretch()
        layout.addWidget(remove_btn)
        
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
        if action_type == "click":
            self.params_layout.addWidget(QLabel("Button:"))
            button_combo = QComboBox()
            button_combo.addItems(["left", "middle", "right"])
            button_combo.setCurrentText(self.action_data.get("button", "left"))
            button_combo.currentTextChanged.connect(
                lambda text, key="button": self.update_action_param(key, text))
            self.params_layout.addWidget(button_combo)
            
            self.params_layout.addWidget(QLabel("Clicks:"))
            clicks_spin = QSpinBox()
            clicks_spin.setRange(1, 10)
            clicks_spin.setValue(self.action_data.get("clicks", 1))
            clicks_spin.valueChanged.connect(
                lambda value, key="clicks": self.update_action_param(key, value))
            self.params_layout.addWidget(clicks_spin)

            # --- Region selection for random click ---
            region_btn = QPushButton("Select Region")
            region_btn.clicked.connect(self.select_region)
            self.params_layout.addWidget(region_btn)
            self.region_label = QLabel()
            self.params_layout.addWidget(self.region_label)
            self.update_region_label()
            
        elif action_type in ["type", "key_press"]:
            param_name = "text" if action_type == "type" else "key"
            self.params_layout.addWidget(QLabel(f"{param_name.title()}: "))
            text_edit = QLineEdit(self.action_data.get(param_name, ""))
            text_edit.textChanged.connect(
                lambda text, key=param_name: self.update_action_param(key, text))
            self.params_layout.addWidget(text_edit)
            
        elif action_type == "wait":
            self.params_layout.addWidget(QLabel("Seconds:"))
            seconds_spin = QDoubleSpinBox()
            seconds_spin.setRange(0.1, 60.0)
            seconds_spin.setValue(self.action_data.get("seconds", 1.0))
            seconds_spin.setSingleStep(0.1)
            seconds_spin.valueChanged.connect(
                lambda value, key="seconds": self.update_action_param(key, value))
            self.params_layout.addWidget(seconds_spin)
            
        elif action_type in ["move", "move_to"]:
            # Add X coordinate
            self.params_layout.addWidget(QLabel("X:"))
            x_spin = QSpinBox()
            x_spin.setRange(0, 10000)
            x_spin.setValue(self.action_data.get("x", 0))
            x_spin.valueChanged.connect(
                lambda value, key="x": self.update_action_param(key, value))
            self.params_layout.addWidget(x_spin)
            
            # Add Y coordinate
            self.params_layout.addWidget(QLabel("Y:"))
            y_spin = QSpinBox()
            y_spin.setRange(0, 10000)
            y_spin.setValue(self.action_data.get("y", 0))
            y_spin.valueChanged.connect(
                lambda value, key="y": self.update_action_param(key, value))
            self.params_layout.addWidget(y_spin)
            
            # Add duration
            self.params_layout.addWidget(QLabel("Duration (s):"))
            duration_spin = QDoubleSpinBox()
            duration_spin.setRange(0.0, 10.0)
            duration_spin.setSingleStep(0.1)
            duration_spin.setValue(self.action_data.get("duration", 0.0))
            duration_spin.valueChanged.connect(
                lambda value, key="duration": self.update_action_param(key, value))
            self.params_layout.addWidget(duration_spin)
        
        elif action_type == "scroll":
            self.params_layout.addWidget(QLabel("Pixels:"))
            pixels_spin = QSpinBox()
            pixels_spin.setRange(-1000, 1000)
            pixels_spin.setValue(self.action_data.get("pixels", 0))
            pixels_spin.valueChanged.connect(
                lambda value, key="pixels": self.update_action_param(key, value))
            self.params_layout.addWidget(pixels_spin)
            
        elif action_type == "click_and_hold":
            # Add button selection
            self.params_layout.addWidget(QLabel("Button:"))
            button_combo = QComboBox()
            button_combo.addItems(["left", "middle", "right"])
            button_combo.setCurrentText(self.action_data.get("button", "left"))
            button_combo.currentTextChanged.connect(
                lambda text, key="button": self.update_action_param(key, text))
            self.params_layout.addWidget(button_combo)
            
            # Add duration control
            self.params_layout.addWidget(QLabel("Hold for (s):"))
            duration_spin = QDoubleSpinBox()
            duration_spin.setRange(0.1, 60.0)
            duration_spin.setSingleStep(0.1)
            duration_spin.setValue(self.action_data.get("duration", 1.0))
            duration_spin.valueChanged.connect(
                lambda value, key="duration": self.update_action_param(key, value))
            self.params_layout.addWidget(duration_spin)
    
    def update_action_param(self, key: str, value: Any):
        self.action_data[key] = value
    
    def get_action_data(self) -> dict:
        action_type = self.type_combo.currentText()
        self.action_data["type"] = action_type
        return self.action_data
    
    def remove_self(self):
        # Try to find the parent StepEditor that has the remove_action method
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
        
        header_layout.addWidget(self.step_label)
        header_layout.addStretch()
        header_layout.addWidget(self.remove_btn)
        
        # Template selection and region group
        template_group = QGroupBox("Template")
        template_layout = QVBoxLayout(template_group)
        
        # First row: Template selection
        template_select_layout = QHBoxLayout()
        
        self.template_combo = QComboBox()
        self.template_combo.addItems([""] + self.templates)
        
        # Add region selection button
        self.select_region_btn = QPushButton("Select Search Region")
        self.select_region_btn.clicked.connect(self.select_search_region)
        
        template_select_layout.addWidget(QLabel("Find:"))
        template_select_layout.addWidget(self.template_combo, 1)  # Allow combo box to expand
        template_select_layout.addWidget(self.select_region_btn)
        
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
        
        # Actions group
        actions_group = QGroupBox("Actions")
        actions_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        actions_layout = QVBoxLayout(actions_group)
        
        # Scroll area for actions
        actions_scroll = QScrollArea()
        actions_scroll.setWidgetResizable(True)
        actions_scroll.setFrameShape(QFrame.Shape.NoFrame)
        
        # Container for action widgets
        self.actions_container = QWidget()
        self.actions_layout = QVBoxLayout(self.actions_container)
        self.actions_layout.setSpacing(5)
        self.actions_layout.setContentsMargins(0, 0, 5, 0)
        self.actions_layout.addStretch()  # Add stretch to push actions to top
        
        actions_scroll.setWidget(self.actions_container)
        actions_layout.addWidget(actions_scroll)
        
        # Add action buttons
        btn_frame = QFrame()
        btn_frame.setFrameShape(QFrame.Shape.StyledPanel)
        btn_layout = QHBoxLayout(btn_frame)
        btn_layout.setContentsMargins(5, 5, 5, 5)
        
        for action_type in ActionType:
            btn = QPushButton(f"Add {action_type.value}")
            btn.clicked.connect(
                lambda checked, at=action_type.value: self.add_action({"type": at}))
            btn_layout.addWidget(btn)
        
        actions_layout.addWidget(btn_frame)
        
        # Add existing actions
        for action in self.step_data.get("actions", []):
            self.add_action(action)
        
        # Add widgets to main layout
        main_layout.addLayout(header_layout)
        main_layout.addWidget(template_group)
        main_layout.addWidget(actions_group, 1)  # Allow actions to expand
        
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
    
    def remove_self(self):
        """Remove this step from the sequence."""
        if hasattr(self.parent_sequence, 'remove_step'):
            self.parent_sequence.remove_step(self)
    
    def add_action(self, action_data: dict):
        action_editor = ActionEditor(action_data, self)
        # Insert before the stretch at the end
        self.actions_layout.insertWidget(self.actions_layout.count() - 1, action_editor)
        self.action_widgets.append(action_editor)
        return action_editor
    
    def remove_action(self, action_widget):
        if action_widget in self.action_widgets:
            # Remove from layout
            self.actions_layout.removeWidget(action_widget)
            # Remove from list
            self.action_widgets.remove(action_widget)
            # Delete the widget
            action_widget.setParent(None)
            action_widget.deleteLater()
    
    def select_search_region(self):
        """Open a dialog to select a search region for this step."""
        dialog = ScreenCaptureDialog(self)
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
    
    def get_step_data(self) -> dict:
        step_data = {
            "find": self.template_combo.currentText(),
            "required": self.required_check.isChecked(),
            "confidence": round(self.confidence_spin.value(), 2),  # Round to 2 decimal places
            "timeout": self.timeout_spin.value(),
            "actions": [w.get_action_data() for w in self.action_widgets],
            "loop_count": self.step_loop_spin.value()
        }
        if self.search_region:
            step_data["search_region"] = self.search_region
        return step_data

class SequenceEditor(QGroupBox):
    """Widget to edit a sequence of steps."""
    def __init__(self, sequence_data: Optional[dict] = None, templates: List[str] = None, parent=None):
        super().__init__(parent)
        self.sequence_data = sequence_data or {"name": "New Sequence", "steps": []}
        self.templates = templates or []
        self.step_widgets = []
        # Failsafe image folder
        self.failsafe_images_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "failsafe_images")
        os.makedirs(self.failsafe_images_dir, exist_ok=True)
        self.init_ui()
    
    def init_ui(self):
        # Main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Sequence header with name and add button
        header_layout = QHBoxLayout()
        
        # Sequence name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Name:"))
        self.name_edit = QLineEdit(self.sequence_data.get("name", "New Sequence"))
        name_layout.addWidget(self.name_edit, 1)  # Allow name field to expand
        
        # Add step button
        self.add_step_btn = QPushButton("Add Step")
        self.add_step_btn.clicked.connect(self.add_step)
        
        # Add to header
        header_layout.addLayout(name_layout)
        header_layout.addWidget(self.add_step_btn, 0, Qt.AlignmentFlag.AlignRight)
        
        # Scroll area for steps
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        
        # Container widget for steps
        self.steps_container = QWidget()
        self.steps_layout = QVBoxLayout(self.steps_container)
        self.steps_layout.setSpacing(10)
        self.steps_layout.setContentsMargins(0, 0, 10, 0)
        self.steps_layout.addStretch()  # Add stretch to push steps to top
        
        # Set up the scroll area
        self.scroll_area.setWidget(self.steps_container)
        
        # Add widgets to main layout
        main_layout.addLayout(header_layout)
        main_layout.addWidget(self.scroll_area, 1)  # 1 is stretch factor
        
        # Add existing steps
        for step in self.sequence_data.get("steps", []):
            self.add_step(step)
        
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
        
        # --- FAILSAFE SECTION ---
        failsafe_group = QGroupBox("Failsafe")
        failsafe_layout = QVBoxLayout(failsafe_group)

        # Enable Failsafe checkbox
        self.failsafe_enable_checkbox = QCheckBox("Enable Failsafe")
        self.failsafe_enable_checkbox.setChecked(bool(self.sequence_data.get('failsafe')))
        self.failsafe_enable_checkbox.toggled.connect(self.toggle_failsafe_ui)
        failsafe_layout.addWidget(self.failsafe_enable_checkbox)

        # Row: Image picker and preview
        fs_img_layout = QHBoxLayout()
        self.failsafe_path_edit = QLineEdit()
        self.failsafe_path_edit.setReadOnly(True)
        self.failsafe_preview = QLabel()
        self.failsafe_preview.setFixedSize(100, 80)
        self.failsafe_preview.setStyleSheet("border: 1px solid #ccc;")
        self.update_failsafe_preview()
        fs_img_layout.addWidget(QLabel("Image:"))
        fs_img_layout.addWidget(self.failsafe_path_edit)
        fs_img_layout.addWidget(self.failsafe_preview)
        fs_img_layout.addStretch()
        # Buttons
        self.fs_capture_btn = QPushButton("Capture")
        self.fs_capture_btn.clicked.connect(self.capture_failsafe_image)
        self.fs_select_btn = QPushButton("Select File")
        self.fs_select_btn.clicked.connect(self.select_failsafe_image)
        fs_img_layout.addWidget(self.fs_capture_btn)
        fs_img_layout.addWidget(self.fs_select_btn)
        failsafe_layout.addLayout(fs_img_layout)

        # Row: Step index selector
        fs_step_layout = QHBoxLayout()
        fs_step_layout.addWidget(QLabel("Jump to Step:"))
        self.fs_step_combo = QComboBox()
        self.update_failsafe_step_combo()
        fs_step_layout.addWidget(self.fs_step_combo)
        fs_step_layout.addStretch()
        failsafe_layout.addLayout(fs_step_layout)

        # Row: Confidence
        fs_conf_layout = QHBoxLayout()
        fs_conf_layout.addWidget(QLabel("Confidence:"))
        self.fs_conf_spin = QDoubleSpinBox()
        self.fs_conf_spin.setRange(0.1, 1.0)
        self.fs_conf_spin.setSingleStep(0.05)
        self.fs_conf_spin.setValue(self.sequence_data.get('failsafe', {}).get('confidence', 0.9))
        self.fs_conf_spin.setDecimals(2)
        fs_conf_layout.addWidget(self.fs_conf_spin)
        fs_conf_layout.addStretch()
        failsafe_layout.addLayout(fs_conf_layout)

        # Row: Region selection
        fs_region_layout = QHBoxLayout()
        self.fs_region_btn = QPushButton("Select Region")
        self.fs_region_btn.clicked.connect(self.select_failsafe_region)
        self.fs_region_label = QLabel("Region: Full Screen")
        fs_region_layout.addWidget(self.fs_region_btn)
        fs_region_layout.addWidget(self.fs_region_label)
        fs_region_layout.addStretch()
        failsafe_layout.addLayout(fs_region_layout)
        self.failsafe_region = None  # (x, y, w, h) or None

        main_layout.addWidget(failsafe_group)

        # Load failsafe config if present
        self.load_failsafe_config()
        # Set failsafe UI enabled/disabled based on checkbox
        self.toggle_failsafe_ui(self.failsafe_enable_checkbox.isChecked())
    
    def update_failsafe_preview(self):
        path = self.failsafe_path_edit.text()
        if path and os.path.exists(path):
            pixmap = QPixmap(path)
            if not pixmap.isNull():
                self.failsafe_preview.setPixmap(
                    pixmap.scaled(self.failsafe_preview.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                 Qt.TransformationMode.SmoothTransformation))
                return
        self.failsafe_preview.setText("No preview")

    def update_failsafe_step_combo(self):
        self.fs_step_combo.clear()
        for i, step in enumerate(self.sequence_data.get('steps', [])):
            name = step.get('find', f"Step {i+1}")
            self.fs_step_combo.addItem(f"{i+1}: {name}", i)
        # Set current index if present
        fs = self.sequence_data.get('failsafe', {})
        goto = fs.get('goto_step')
        if goto is not None and 0 <= goto < self.fs_step_combo.count():
            self.fs_step_combo.setCurrentIndex(goto)

    def capture_failsafe_image(self):
        dialog = ScreenCaptureDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            rect = dialog.get_capture_rect()
            if rect and rect.isValid():
                from PIL import ImageGrab
                screenshot = ImageGrab.grab(bbox=(rect.x(), rect.y(), rect.x()+rect.width(), rect.y()+rect.height()))
                # Prompt for custom name
                from PyQt6.QtWidgets import QInputDialog
                name, ok = QInputDialog.getText(self, "Failsafe Image Name", "Enter a name for this failsafe image:")
                if not ok or not name.strip():
                    return  # Cancelled or empty name
                # Sanitize name
                import re
                safe_name = re.sub(r'[^a-zA-Z0-9_\-]', '_', name.strip())
                fname = f"{safe_name}.png"
                path = os.path.join(self.failsafe_images_dir, fname)
                screenshot.save(path, "PNG")
                self.failsafe_path_edit.setText(path)
                self.update_failsafe_preview()

    def select_failsafe_image(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Select Failsafe Image", "", "Images (*.png *.jpg *.jpeg *.bmp)")
        if file_path:
            # Copy to failsafe_images dir
            import shutil
            fname = os.path.basename(file_path)
            dest_path = os.path.join(self.failsafe_images_dir, fname)
            if not os.path.exists(dest_path):
                shutil.copy2(file_path, dest_path)
            self.failsafe_path_edit.setText(dest_path)
            self.update_failsafe_preview()

    def load_failsafe_config(self):
        fs = self.sequence_data.get('failsafe', {})
        path = fs.get('template')
        if path:
            # If relative, make absolute
            if not os.path.isabs(path):
                path = os.path.join(self.failsafe_images_dir, os.path.basename(path))
            self.failsafe_path_edit.setText(path)
            self.update_failsafe_preview()
        goto = fs.get('goto_step')
        if goto is not None and self.fs_step_combo.count() > goto:
            self.fs_step_combo.setCurrentIndex(goto)
        conf = fs.get('confidence')
        if conf:
            self.fs_conf_spin.setValue(conf)
        region = fs.get('region')
        if region and isinstance(region, (list, tuple)) and len(region) == 4:
            self.failsafe_region = tuple(region)
            x, y, w, h = self.failsafe_region
            self.fs_region_label.setText(f"Region: ({x}, {y}, {w}x{h})")
        else:
            self.failsafe_region = None
            self.fs_region_label.setText("Region: Full Screen")

    def add_step(self, step_data: Optional[dict] = None):
        """Add a new step to the sequence."""
        try:
            # Ensure step_data has the required structure with at least one default action
            if step_data is None or not isinstance(step_data, dict):
                step_data = {
                    "find": "",
                    "required": True,
                    "timeout": 10,
                    "actions": [{"type": "click"}]
                }
            elif not step_data.get('actions') or not isinstance(step_data['actions'], list):
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
        
        # Insert the new step before the stretch at the end
        self.steps_layout.insertWidget(self.steps_layout.count() - 1, step_editor)
        self.step_widgets.append(step_editor)
        
        # Update the step numbers
        self.update_step_numbers()
        
        # Ensure the new step is visible
        QTimer.singleShot(100, lambda: self.scroll_to_bottom())
        
        return step_editor
    
    def scroll_to_bottom(self):
        """Scroll the scroll area to the bottom."""
        scrollbar = self.scroll_area.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        # Force an update to ensure the scroll happens
        self.scroll_area.viewport().update()
    
    def remove_step(self, step_widget):
        """Remove a step from the sequence."""
        if step_widget in self.step_widgets:
            # Ask for confirmation if the step has actions
            if step_widget.action_widgets:
                reply = QMessageBox.question(
                    self, 'Remove Step', 
                    'Are you sure you want to remove this step and all its actions?',
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No
                )
                if reply != QMessageBox.StandardButton.Yes:
                    return
            
            # Remove from layout
            self.steps_layout.removeWidget(step_widget)
            # Remove from list
            self.step_widgets.remove(step_widget)
            # Delete the widget
            step_widget.setParent(None)
            step_widget.deleteLater()
            # Update step numbers
            self.update_step_numbers()
    
    def update_step_numbers(self):
        """Update the step numbers in the UI."""
        for i, step in enumerate(self.step_widgets, 1):
            if hasattr(step, 'step_label'):
                step.step_label.setText(f"Step {i}")
    
    def get_sequence_data(self) -> dict:
        """Get the sequence data including all steps and their actions."""
        data = {
            "name": self.name_edit.text(),
            "steps": [w.get_step_data() for w in self.step_widgets]
        }
        # Failsafe config (only if enabled)
        if self.failsafe_enable_checkbox.isChecked():
            path = self.failsafe_path_edit.text().strip()
            goto = self.fs_step_combo.currentData()
            conf = self.fs_conf_spin.value()
            region = self.failsafe_region
            if path and goto is not None:
                # Store relative path if in failsafe_images
                rel_path = os.path.relpath(path, os.path.dirname(os.path.abspath(__file__)))
                failsafe_dict = {
                    "template": rel_path,
                    "goto_step": goto,
                    "confidence": conf
                }
                if region:
                    failsafe_dict["region"] = list(region)
                data["failsafe"] = failsafe_dict
        return data

    def select_failsafe_region(self):
        dialog = ScreenCaptureDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            rect = dialog.get_capture_rect()
            if rect and rect.isValid():
                self.failsafe_region = (rect.x(), rect.y(), rect.width(), rect.height())
                self.fs_region_label.setText(f"Region: ({rect.x()}, {rect.y()}, {rect.width()}x{rect.height()})")
            else:
                self.failsafe_region = None
                self.fs_region_label.setText("Region: Full Screen")

    def toggle_failsafe_ui(self, enabled):
        # Enable/disable all failsafe config widgets except the enable checkbox
        for widget in [self.failsafe_path_edit, self.failsafe_preview, self.fs_capture_btn, self.fs_select_btn,
                      self.fs_step_combo, self.fs_conf_spin, self.fs_region_btn, self.fs_region_label]:
            widget.setEnabled(enabled)

class MainWindow(QMainWindow):
    """Main application window."""
    def __init__(self):
        super().__init__()
        self.config = {}
        self.config_path = "config.json"
        self.bot = ImageDetectionBot()
        self.search_region = None  # Will store the search region (x, y, width, height)
        self.worker = None  # Will store the worker thread
        self.f8_pressed = False  # Track F8 key state
        
        # Initialize images directory
        self.images_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")
        os.makedirs(self.images_dir, exist_ok=True)
        
        self.init_ui()
        self.load_config()
        
        # Set up F8 key monitoring
        self.f8_timer = QTimer(self)
        self.f8_timer.timeout.connect(self.check_f8_key)
        self.f8_timer.start(100)  # Check every 100ms
    
    def init_ui(self):
        """Initialize the UI."""
        self.setWindowTitle("Image Detection Bot")
        self.setGeometry(100, 100, 1200, 800)
        
        # Create a global shortcut for F8 to stop sequences
        self.f8_shortcut = QShortcut(QKeySequence("F8"), self)
        self.f8_shortcut.activated.connect(self.stop_sequence)
        self.f8_shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)
        
        # Create main widget and layout
        main_widget = QWidget()
        main_layout = QVBoxLayout(main_widget)
        
        # Toolbar
        toolbar = QToolBar()
        self.addToolBar(toolbar)
        
        # Region selection action
        self.region_action = QAction("Select Region", self)
        self.region_action.setCheckable(True)
        self.region_action.toggled.connect(self.toggle_region_selection)
        toolbar.addAction(self.region_action)
        
        # Add separator
        toolbar.addSeparator()
        
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
        content_layout = QHBoxLayout(content_widget)
        content_layout.setContentsMargins(0, 0, 0, 0)
        
        main_layout.addWidget(toolbar)
        main_layout.addWidget(content_widget)
        
        # Store the main content layout for adding widgets later
        self.main_content_layout = content_layout
        
        # Left sidebar
        sidebar = QWidget()
        sidebar.setFixedWidth(250)
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(0, 0, 0, 0)
        
        # Templates section
        templates_group = QGroupBox("Templates")
        templates_layout = QVBoxLayout()
        
        self.templates_list = QListWidget()
        self.templates_list.currentItemChanged.connect(self.on_template_selected)
        
        # Button layout for Add/Delete template
        template_btns_layout = QHBoxLayout()
        
        add_template_btn = QPushButton("Add")
        add_template_btn.clicked.connect(self.add_template)
        
        self.delete_template_btn = QPushButton("Delete")
        self.delete_template_btn.clicked.connect(self.delete_template)
        self.delete_template_btn.setEnabled(False)  # Disable by default, enable when a template is selected
        
        template_btns_layout.addWidget(add_template_btn)
        template_btns_layout.addWidget(self.delete_template_btn)
        template_btns_layout.addStretch()
        
        templates_layout.addWidget(self.templates_list)
        templates_layout.addLayout(template_btns_layout)
        templates_group.setLayout(templates_layout)
        
        # Sequences section
        sequences_group = QGroupBox("Sequences")
        sequences_layout = QVBoxLayout()
        
        self.sequences_list = QListWidget()
        self.sequences_list.currentItemChanged.connect(self.on_sequence_selected)
        
        # Button layout for Add/Remove sequence
        sequence_btns_layout = QHBoxLayout()
        
        add_sequence_btn = QPushButton("Add")
        add_sequence_btn.clicked.connect(self.add_sequence)
        
        self.remove_sequence_btn = QPushButton("Remove")
        self.remove_sequence_btn.clicked.connect(self.remove_sequence)
        self.remove_sequence_btn.setEnabled(False)  # Disable by default, enable when a sequence is selected
        
        sequence_btns_layout.addWidget(add_sequence_btn)
        sequence_btns_layout.addWidget(self.remove_sequence_btn)
        sequence_btns_layout.addStretch()
        
        sequences_layout.addLayout(sequence_btns_layout)
        
        # Loop controls
        loop_control_layout = QHBoxLayout()
        
        # Loop checkbox
        self.loop_checkbox = QCheckBox("Loop Sequence")
        self.loop_checkbox.toggled.connect(self.on_loop_toggled)
        
        # Loop count spinner
        self.loop_count_label = QLabel("Times:")
        self.loop_count_spin = QSpinBox()
        self.loop_count_spin.setMinimum(1)
        self.loop_count_spin.setMaximum(9999)
        self.loop_count_spin.setValue(1)
        self.loop_count_spin.setEnabled(False)  # Disabled by default, enabled when loop is checked
        
        # Add widgets to loop control layout
        loop_control_layout.addWidget(self.loop_checkbox)
        loop_control_layout.addWidget(self.loop_count_label)
        loop_control_layout.addWidget(self.loop_count_spin)
        loop_control_layout.addStretch()
        
        # Run sequence button
        run_sequence_btn = QPushButton("Run Sequence")
        run_sequence_btn.clicked.connect(self.run_sequence)
        
        # Add widgets to sequences layout
        sequences_layout.addWidget(self.sequences_list)
        sequences_layout.addLayout(loop_control_layout)
        sequences_layout.addWidget(run_sequence_btn)
        sequences_group.setLayout(sequences_layout)
        
        # Add sections to sidebar
        sidebar_layout.addWidget(templates_group)
        sidebar_layout.addWidget(sequences_group)
        
        # Add sidebar to main layout
        splitter = QSplitter()
        splitter.addWidget(sidebar)
        
        # Main content area
        self.content_stack = QStackedWidget()
        splitter.addWidget(self.content_stack)
        splitter.setStretchFactor(1, 1)
        
        self.main_content_layout.addWidget(splitter)
        
        # Welcome screen
        welcome_widget = QWidget()
        welcome_layout = QVBoxLayout(welcome_widget)
        welcome_layout.addWidget(QLabel("Welcome to Image Detection Bot"))
        welcome_layout.addStretch()
        
        # Template editor
        self.template_editor = QWidget()
        template_editor_layout = QVBoxLayout(self.template_editor)
        template_editor_layout.addWidget(QLabel("Template Editor"))
        
        # Sequence editor
        self.sequence_editor = QWidget()
        sequence_editor_layout = QVBoxLayout(self.sequence_editor)
        sequence_editor_layout.addWidget(QLabel("Sequence Editor"))
        
        # Add widgets to stack
        self.content_stack.addWidget(welcome_widget)
        self.content_stack.addWidget(self.template_editor)
        self.content_stack.addWidget(self.sequence_editor)
        
        # Set initial content
        self.content_stack.setCurrentWidget(welcome_widget)
        self.setCentralWidget(main_widget)
        
        # Create menu bar
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("&File")
        
        new_action = QAction("&New", self)
        new_action.setShortcut("Ctrl+N")
        new_action.triggered.connect(self.new_config)
        file_menu.addAction(new_action)
        
        open_action = QAction("&Open...", self)
        open_action.setShortcut("Ctrl+O")
        open_action.triggered.connect(self.open_config)
        file_menu.addAction(open_action)
        
        save_action = QAction("&Save", self)
        save_action.setShortcut("Ctrl+S")
        save_action.triggered.connect(self.save_config)
        file_menu.addAction(save_action)
        
        save_as_action = QAction("Save &As...", self)
        save_as_action.triggered.connect(self.save_config_as)
        file_menu.addAction(save_as_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut("Alt+F4")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Run menu
        run_menu = menubar.addMenu("&Run")
        
        self.run_action = QAction("Run Sequence", self)
        self.run_action.setShortcut("F5")
        self.run_action.triggered.connect(self.run_sequence)
        run_menu.addAction(self.run_action)
        
        self.stop_action = QAction("Stop Sequence", self)
        self.stop_action.setShortcut("F8")
        self.stop_action.triggered.connect(self.stop_sequence)
        self.stop_action.setEnabled(False)  # Disabled by default
        run_menu.addAction(self.stop_action)
        
        # Help menu
        help_menu = menubar.addMenu("&Help")
        
        about_action = QAction("&About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
        
        # Add global shortcut for F8 to stop sequence
        self.stop_shortcut = QShortcut(QKeySequence("F8"), self)
        self.stop_shortcut.activated.connect(self.stop_sequence)
    
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
                "sequences": []
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
            try:
                with open(file_path, 'r') as f:
                    self.config = json.load(f)
                self.config_path = file_path
                self.update_ui_from_config()
                self.statusBar().showMessage(f"Loaded configuration from {file_path}")
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
            if os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    self.config = json.load(f)
                self.config_path = file_path
                
                # Load search region if it exists
                if 'search_region' in self.config:
                    self.search_region = self.config['search_region']
                    if isinstance(self.search_region, (list, tuple)) and len(self.search_region) == 4:
                        x, y, w, h = self.search_region
                        self.region_status.setText(f"Region: ({x}, {y}, {w}, {h})")
                    else:
                        self.search_region = None
                        self.region_status.setText("Region: Full Screen")
                
                # Update template paths to be absolute
                if 'templates' in self.config:
                    updated_templates = {}
                    for name, path in self.config['templates'].items():
                        # If path is relative, make it absolute relative to the config file
                        if not os.path.isabs(path):
                            abs_path = os.path.join(os.path.dirname(os.path.abspath(file_path)), path)
                            if os.path.exists(abs_path):
                                path = abs_path
                        updated_templates[name] = path
                    self.config['templates'] = updated_templates
                
                # Update UI and load templates into the bot
                self.update_ui_from_config()
                self.statusBar().showMessage(f"Loaded configuration from {file_path}")
            else:
                self.statusBar().showMessage("No configuration file found, using defaults")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load configuration: {str(e)}")
    
    def save_config(self):
        """Save configuration to file."""
        try:
            # Update config from UI before saving
            self.update_config_from_ui()
            
            # Include search region in config if it exists and is valid
            if (hasattr(self, 'search_region') and 
                self.search_region is not None and 
                isinstance(self.search_region, (list, tuple)) and 
                len(self.search_region) == 4):
                self.config['search_region'] = list(self.search_region)  # Ensure it's a list for JSON serialization
            else:
                self.config.pop('search_region', None)
            
            # Ensure templates directory exists
            os.makedirs(self.images_dir, exist_ok=True)
            
            # Ensure all template files exist and update their paths if needed
            if 'templates' in self.config:
                updated_templates = {}
                for name, path in self.config['templates'].items():
                    if not path:  # Skip empty paths
                        continue
                        
                    # Convert to absolute path for checking existence
                    abs_path = path if os.path.isabs(path) else os.path.join(os.path.dirname(os.path.abspath(self.config_path)), path)
                    
                    if os.path.exists(abs_path):
                        # If the file is in the images directory, use relative path
                        if os.path.commonpath([os.path.normpath(abs_path), os.path.normpath(self.images_dir)]) == os.path.normpath(self.images_dir):
                            rel_path = os.path.relpath(abs_path, os.path.dirname(os.path.abspath(self.config_path)))
                            updated_templates[name] = rel_path
                        else:
                            updated_templates[name] = abs_path
                self.config['templates'] = updated_templates
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(os.path.abspath(self.config_path)), exist_ok=True)
            
            # Save to file
            with open(self.config_path, 'w') as f:
                json.dump(self.config, f, indent=4)
                
            self.statusBar().showMessage(f"Configuration saved to {self.config_path}")
            return True
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save configuration: {str(e)}")
            return False
    
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
    
    def update_ui_from_config(self):
        """Update UI elements from current config."""
        logger.info("Updating UI from config...")
        
        # Clear existing templates from the bot
        if hasattr(self, 'bot'):
            logger.info("Clearing existing templates from bot")
            self.bot.templates = {}
            
        # Update templates list and load them into the bot
        self.templates_list.clear()
        templates = self.config.get('templates', {})
        logger.info(f"Found {len(templates)} templates in config")
        
        for name, path in templates.items():
            logger.info(f"Processing template: {name} -> {path}")
            
            # Add to UI
            self.templates_list.addItem(name)
            
            # Load into bot if the file exists
            if hasattr(self, 'bot'):
                if os.path.exists(path):
                    logger.info(f"Loading template '{name}' from {path}")
                    success = self.bot.load_template(name, path)
                    if success:
                        logger.info(f"Successfully loaded template '{name}'")
                    else:
                        logger.error(f"Failed to load template '{name}' from {path}")
                else:
                    logger.error(f"Template file not found: {path}")
        
        # Update sequences list
        self.sequences_list.clear()
        for seq in self.config.get('sequences', []):
            self.sequences_list.addItem(seq.get('name', 'Unnamed Sequence'))
    
    def update_config_from_ui(self):
        """Update config from UI elements."""
        # Update templates
        if hasattr(self, 'template_editor'):
            template_data = self.template_editor.get_data()
            if template_data['name']:
                self.config['templates'][template_data['name']] = template_data['path']
        
        # Update sequences
        if hasattr(self, 'sequence_editor_widget'):
            sequence_data = self.sequence_editor_widget.get_sequence_data()
            if sequence_data['name']:
                # Find and update the sequence in the config
                for i, seq in enumerate(self.config.get('sequences', [])):
                    if seq.get('name') == sequence_data['name']:
                        self.config['sequences'][i] = sequence_data
                        break
                else:
                    # Add new sequence if not found
                    self.config.setdefault('sequences', []).append(sequence_data)
    
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
                
                # If the file is in the temp directory, move it to the main images directory
                if os.path.dirname(path) != self.images_dir:
                    new_path = os.path.join(self.images_dir, f"{name}{os.path.splitext(path)[1]}")
                    shutil.move(path, new_path)
                    path = new_path
                
                # Make sure the path is absolute
                if not os.path.isabs(path):
                    path = os.path.join(os.path.dirname(os.path.abspath(self.config_path)), path)
                
                # Add to config with relative path
                rel_path = os.path.relpath(path, os.path.dirname(os.path.abspath(self.config_path)))
                self.config.setdefault('templates', {})[name] = rel_path
                
                # Load the template into the bot
                if hasattr(self, 'bot') and os.path.exists(path):
                    success = self.bot.load_template(name, path)
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
                
                # Save config
                self.save_config()
                
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
                    self.content_stack.removeWidget(self.current_template_widget)
                    self.current_template_widget.deleteLater()
                    del self.current_template_widget
                    self.content_stack.setCurrentIndex(0)  # Show welcome screen
                
                # Remove from bot's template cache
                if hasattr(self, 'bot') and hasattr(self.bot, 'templates') and template_name in self.bot.templates:
                    del self.bot.templates[template_name]
                
                # Update sequence editor's template list if it exists
                if hasattr(self, 'sequence_editor_widget'):
                    self.sequence_editor_widget.templates = list(self.config.get('templates', {}).keys())
                
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
            if hasattr(self, 'sequence_editor_widget') and self.sequence_editor_widget.sequence_data.get('name') == sequence_name:
                self.main_content_layout.removeWidget(self.current_sequence_widget)
                self.current_sequence_widget.deleteLater()
                del self.sequence_editor_widget
                del self.current_sequence_widget
            
            # Save changes
            self.save_config()
            self.statusBar().showMessage(f"Sequence '{sequence_name}' deleted")
    
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
            template_path = self.config['templates'][template_name]
            self.template_editor = TemplatePreview(template_name, template_path)
            
            # Remove the old editor if it exists
            if hasattr(self, 'current_template_widget'):
                self.content_stack.removeWidget(self.current_template_widget)
                self.current_template_widget.deleteLater()
            
            # Add the new editor
            self.current_template_widget = QWidget()
            layout = QVBoxLayout(self.current_template_widget)
            layout.addWidget(self.template_editor)
            
            # Add save button
            save_btn = QPushButton("Save Template")
            save_btn.clicked.connect(self.save_current_template)
            layout.addWidget(save_btn)
            
            # Add to stack and show
            self.content_stack.addWidget(self.current_template_widget)
            self.content_stack.setCurrentWidget(self.current_template_widget)
    
    def on_loop_toggled(self, checked):
        """Handle loop checkbox state change."""
        if hasattr(self, 'loop_count_spin'):
            self.loop_count_spin.setEnabled(checked)
            self.loop_count_label.setEnabled(checked)
    
    def save_config(self):
        """Save the current configuration to a file."""
        try:
            with open(self.config_path, 'w') as f:
                json.dump(self.config, f, indent=4)
            return True
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save configuration: {str(e)}")
            return False
    
    def save_current_sequence(self):
        """Save the current sequence data including loop settings."""
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
    
    def on_sequence_selected(self, current, previous):
        """Handle sequence selection change."""
        # Enable/disable remove button based on selection
        self.remove_sequence_btn.setEnabled(current is not None)
        
        if not current:
            return
            
        sequence_name = current.text()
        sequence = next((s for s in self.config.get('sequences', []) if s.get('name') == sequence_name), None)
        
        if sequence:
            # Update loop controls state
            if hasattr(self, 'loop_checkbox'):
                loop_enabled = sequence.get('loop', False)
                self.loop_checkbox.setChecked(loop_enabled)
                if hasattr(self, 'loop_count_spin'):
                    self.loop_count_spin.setValue(sequence.get('loop_count', 1))
                    self.loop_count_spin.setEnabled(loop_enabled)
                    self.loop_count_label.setEnabled(loop_enabled)
                
            # Create new sequence editor widget
            self.sequence_editor_widget = SequenceEditor(
                sequence, 
                list(self.config.get('templates', {}).keys()), 
                self
            )
            
            # Clear the layout before adding new widget
            while self.main_content_layout.count() > 1:  # Keep the first widget (sidebar)
                item = self.main_content_layout.takeAt(1)
                if item.widget():
                    item.widget().deleteLater()
            
            # Add the new widget to the layout
            self.main_content_layout.addWidget(self.sequence_editor_widget, 1)
            self.current_sequence_widget = self.sequence_editor_widget
    
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
        
        # --- Ensure failsafe is only present if enabled ---
        if hasattr(self, 'sequence_editor_widget') and hasattr(self.sequence_editor_widget, 'failsafe_enable_checkbox'):
            if not self.sequence_editor_widget.failsafe_enable_checkbox.isChecked() and 'failsafe' in sequence:
                sequence.pop('failsafe', None)
        
        # Get loop settings
        should_loop = sequence.get('loop', False)
        loop_count = sequence.get('loop_count', 1)
        
        # Disable UI during execution
        self.setEnabled(False)
        status_text = f"Running sequence: {sequence_name}"
        if should_loop:
            if loop_count > 1:
                status_text += f" (Looping {loop_count} times)"
            else:
                status_text += " (Looping)"
        self.statusBar().showMessage(status_text)
        
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
        self.worker = BotWorker(self.bot, sequence, should_loop, loop_count)
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
            self.statusBar().showMessage(data['status'])
        if 'progress' in data:
            # Update progress bar if you have one
            pass
            
    def on_worker_finished(self):
        """Handle worker completion."""
        self.setEnabled(True)
        self.stop_action.setEnabled(False)
        self.statusBar().showMessage("Sequence completed successfully")
        
    def on_worker_error(self, error: str):
        """Handle worker errors."""
        self.setEnabled(True)
        QMessageBox.critical(self, "Error", error)
        self.statusBar().showMessage(f"Error: {error}")
        
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
