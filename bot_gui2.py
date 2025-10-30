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
                            QHeaderView, QAbstractItemView, QTabWidget, QGridLayout, QSlider, QLayout)
from PyQt6.QtCore import Qt, QSize, QTimer, pyqtSignal, QThread, QObject, QPoint, QRect, QEvent, QDateTime, QMimeData
from PyQt6.QtGui import (QAction, QIcon, QPixmap, QImage, QPainter, QPen, QColor, 
                        QScreen, QGuiApplication, QKeySequence, QShortcut, QKeyEvent, QDrag)
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
                                    for idx, action_data in enumerate(actions):
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
                                            # --- NEW LOGIC: Only use template position for the first action, or for move_to/click with region ---
                                            use_template_position = False
                                            if idx == 0:
                                                use_template_position = True
                                            elif action.type == ActionType.MOVE_TO:
                                                use_template_position = True
                                            elif action.type == ActionType.CLICK and getattr(action, 'region', None):
                                                use_template_position = True
                                            # Otherwise, use current mouse position
                                            if action.type == ActionType.MOVE:
                                                if action.x is not None and action.y is not None:
                                                    success = self.bot.execute_action(action)
                                                else:
                                                    self.signals.error.emit("MOVE action requires x and y coordinates")
                                                    should_continue = False
                                                    break
                                            elif use_template_position:
                                                success = self.bot.execute_action_at_position(action, position)
                                            elif action.type == ActionType.CLICK_AND_HOLD:
                                                success = self.bot.execute_action(action)
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

class ActionEditor(QFrame):
    """Widget to edit a single action in a step."""
    remove_requested = pyqtSignal()
    
    def __init__(self, action_data: Optional[dict] = None, parent=None):
        super().__init__(parent)
        self.action_data = action_data or {"type": "click"}
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setStyleSheet("""
            ActionEditor {
                background: #3a3a3a;
                border: 1px solid #4a4a4a;
                border-radius: 4px;
                padding: 8px;
                margin: 2px 0;
            }
            ActionEditor QLabel {
                color: #ffffff;
                padding: 2px 0;
            }
            ActionEditor QComboBox, ActionEditor QLineEdit, 
            ActionEditor QSpinBox, ActionEditor QDoubleSpinBox {
                background: #2a2a2a;
                color: #ffffff;
                border: 1px solid #4a4a4a;
                padding: 4px;
                border-radius: 3px;
                min-width: 80px;
            }
            ActionEditor QPushButton {
                background: #4a4a4a;
                color: #ffffff;
                border: 1px solid #5a5a5a;
                padding: 4px 8px;
                border-radius: 3px;
            }
            ActionEditor QPushButton:hover {
                background: #5a5a5a;
                border-color: #6a6a6a;
            }
            ActionEditor QCheckBox {
                color: #ffffff;
                spacing: 5px;
            }
            ActionEditor:hover {
                border: 1px solid #4a9ff5;
                background: #3f3f3f;
            }
        """)
        self.setMinimumHeight(40)
        self.setAcceptDrops(True)
        self.init_ui()
        
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drag_start_position = event.pos()
        super().mousePressEvent(event)
        
    def mouseMoveEvent(self, event):
        if not (event.buttons() & Qt.MouseButton.LeftButton):
            return
            
        if (event.pos() - self.drag_start_position).manhattanLength() < QApplication.startDragDistance():
            return
            
        # Create the drag object
        drag = QDrag(self)
        mime_data = QMimeData()
        
        # Store the widget's index in the parent's layout
        if hasattr(self, 'step_editor'):
            index = self.step_editor.action_widgets.index(self)
            mime_data.setText(str(index))
            drag.setMimeData(mime_data)
            
            # Start the drag operation
            drag.exec(Qt.DropAction.MoveAction)
        
    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            event.acceptProposedAction()
            
    def dropEvent(self, event):
        if not event.mimeData().hasText():
            return
            
        source_index = int(event.mimeData().text())
        target_index = self.step_editor.action_widgets.index(self)
        
        # Don't drop on self
        if source_index == target_index:
            return
            
        # Get the source widget
        source_widget = self.step_editor.action_widgets[source_index]
        
        # Remove from current position
        self.step_editor.action_widgets.pop(source_index)
        
        # Insert at new position
        if target_index > source_index:
            target_index -= 1
            
        self.step_editor.action_widgets.insert(target_index, source_widget)
        
        # Rebuild the layout
        for i in reversed(range(self.step_editor.actions_layout.count())):
            widget = self.step_editor.actions_layout.itemAt(i).widget()
            if widget and widget != source_widget:  # Don't remove the source widget yet
                widget.setParent(None)
                
        # Re-add all widgets in new order
        for i, widget in enumerate(self.step_editor.action_widgets):
            self.step_editor.actions_layout.insertWidget(i, widget)
            
        # Ensure the source widget is visible
        source_widget.show()
        event.acceptProposedAction()
    
    def init_ui(self):
        layout = QHBoxLayout()
        
        # Action type
        self.type_combo = QComboBox()
        # Add action types directly as strings to match ActionType enum values
        # Removed 'move_to' as it's redundant with 'move'
        action_types = ["click", "move", "right_click", "double_click", "type", "key_press", "wait", "scroll", "click_and_hold"]
        self.type_combo.addItems(action_types)
        if "type" in self.action_data:
            action_type = self.action_data["type"]
            # If the action type is 'move_to', convert it to 'move' for backward compatibility
            if action_type == "move_to":
                action_type = "move"
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
        # Clear previous parameters
        while self.params_layout.count() > 0:
            item = self.params_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        
        action_type = self.type_combo.currentText()
        
        if action_type in ["click", "right_click", "double_click"]:
            # Add button type selector
            self.params_layout.addWidget(QLabel("Button:"))
            button_combo = QComboBox()
            button_combo.addItems(["left", "middle", "right"])
            button_combo.setCurrentText(self.action_data.get("button", "left"))
            button_combo.currentTextChanged.connect(
                lambda text, k="button": self.update_action_param(k, text))
            self.params_layout.addWidget(button_combo)
            
            # Add click count selector
            self.params_layout.addWidget(QLabel("Clicks:"))
            clicks_spin = QSpinBox()
            clicks_spin.setRange(1, 10)
            clicks_spin.setValue(self.action_data.get("clicks", 1))
            clicks_spin.valueChanged.connect(
                lambda value, k="clicks": self.update_action_param(k, value))
            self.params_layout.addWidget(clicks_spin)
            
        elif action_type == "move":
            # Toggle random checkbox
            self.random_checkbox = QCheckBox("Random Position in Region")
            is_random = self.action_data.get("random", False)
            self.random_checkbox.setChecked(is_random)
            self.random_checkbox.toggled.connect(
                lambda checked: self.update_action_param("random", checked) or self.update_params()
            )
            self.params_layout.addWidget(self.random_checkbox)
            
            if is_random:
                # Add random region button and label
                region_btn = QPushButton("Select Region")
                region_btn.clicked.connect(self.select_random_region)
                self.params_layout.addWidget(region_btn)
                
                self.random_region_label = QLabel("Region: Not set")
                self.update_random_region_label()
                self.params_layout.addWidget(self.random_region_label)
            else:
                # Add X and Y coordinates
                for coord in ["x", "y"]:
                    self.params_layout.addWidget(QLabel(f"{coord.upper()}:"))
                    spin = QSpinBox()
                    spin.setRange(0, 9999)
                    spin.setValue(self.action_data.get(coord, 0))
                    spin.valueChanged.connect(
                        lambda value, k=coord: self.update_action_param(k, value))
                    self.params_layout.addWidget(spin)
        
        elif action_type == "type":
            # Add text input
            text_edit = QLineEdit()
            text_edit.setText(self.action_data.get("text", ""))
            text_edit.textChanged.connect(
                lambda text, k="text": self.update_action_param(k, text))
            self.params_layout.addWidget(QLabel("Text:"))
            self.params_layout.addWidget(text_edit)
            
        elif action_type == "key_press":
            # Add key selector
            key_edit = QLineEdit()
            key_edit.setText(self.action_data.get("key", ""))
            key_edit.setPlaceholderText("Press a key...")
            key_edit.keyPressEvent = lambda e: (
                key_edit.setText(QKeySequence(e.key()).toString()),
                self.update_action_param("key", QKeySequence(e.key()).toString())
            )
            self.params_layout.addWidget(QLabel("Key:"))
            self.params_layout.addWidget(key_edit)
            
        elif action_type == "wait":
            # Add wait time selector
            wait_spin = QDoubleSpinBox()
            wait_spin.setRange(0.1, 60.0)
            wait_spin.setValue(self.action_data.get("seconds", 1.0))
            wait_spin.setSingleStep(0.1)
            wait_spin.valueChanged.connect(
                lambda value, k="seconds": self.update_action_param(k, value))
            self.params_layout.addWidget(QLabel("Seconds:"))
            self.params_layout.addWidget(wait_spin)
            
        elif action_type == "scroll":
            # Add scroll amount
            scroll_spin = QSpinBox()
            scroll_spin.setRange(-100, 100)
            scroll_spin.setValue(self.action_data.get("clicks", 1))
            scroll_spin.valueChanged.connect(
                lambda value, k="clicks": self.update_action_param(k, value))
            self.params_layout.addWidget(QLabel("Clicks:"))
            self.params_layout.addWidget(scroll_spin)
            
            # Add direction
            direction_combo = QComboBox()
            direction_combo.addItems(["up", "down", "left", "right"])
            direction_combo.setCurrentText(self.action_data.get("direction", "down"))
            direction_combo.currentTextChanged.connect(
                lambda text, k="direction": self.update_action_param(k, text))
            self.params_layout.addWidget(QLabel("Direction:"))
            self.params_layout.addWidget(direction_combo)
            
        elif action_type == "click_and_hold":
            # Add duration
            duration_spin = QDoubleSpinBox()
            duration_spin.setRange(0.1, 60.0)
            duration_spin.setValue(self.action_data.get("duration", 1.0))
            duration_spin.setSingleStep(0.1)
            duration_spin.valueChanged.connect(
                lambda value, k="duration": self.update_action_param(k, value))
            self.params_layout.addWidget(QLabel("Duration (s):"))
            self.params_layout.addWidget(duration_spin)
    
    def update_action_param(self, key: str, value: Any):
        self.action_data[key] = value
        # If we're updating the random checkbox, update the UI
        if key == 'random':
            self.update_params()
    
    def get_action_data(self) -> dict:
        # Create a new dictionary to avoid modifying the original
        result = self.action_data.copy()
        
        # Always include the action type
        result["type"] = self.type_combo.currentText()
        
        # If we have a random region in the UI, include it
        if hasattr(self, 'random_region_label') and hasattr(self, 'action_data'):
            if 'random_region' in self.action_data:
                result['random_region'] = self.action_data['random_region']
            elif hasattr(self, 'random_region'):
                result['random_region'] = getattr(self, 'random_region', None)
        
        # Always include the random flag if it's set
        if 'random' in self.action_data:
            result['random'] = self.action_data['random']
            
        logger.debug(f"Action data being saved: {result}")
        return result
    
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
    
    def select_random_region(self):
        dialog = ScreenCaptureDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            rect = dialog.get_capture_rect()
            if rect and rect.isValid():
                self.action_data["random"] = True  # Ensure random is enabled
                self.action_data["random_region"] = [rect.x(), rect.y(), rect.width(), rect.height()]
                if hasattr(self, 'random_checkbox') and not self.random_checkbox.isChecked():
                    self.random_checkbox.setChecked(True)
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
        actions_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        actions_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        actions_scroll.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        actions_scroll.setMinimumHeight(200)  # Ensure minimum height for the scroll area
        
        # Container for action widgets with drag and drop support
        self.actions_container = QWidget()
        self.actions_container.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.MinimumExpanding)
        self.actions_layout = QVBoxLayout(self.actions_container)
        self.actions_layout.setSpacing(10)  # Increased spacing between actions
        self.actions_layout.setContentsMargins(5, 5, 5, 5)
        self.actions_layout.setSizeConstraint(QLayout.SizeConstraint.SetMinAndMaxSize)
        
        # Add a container widget for the actions (helps with layout management)
        self.actions_inner_container = QWidget()
        self.actions_inner_layout = QVBoxLayout(self.actions_inner_container)
        self.actions_inner_layout.setSpacing(10)
        self.actions_inner_layout.setContentsMargins(0, 0, 0, 0)
        self.actions_inner_layout.addStretch()
        
        self.actions_layout.addWidget(self.actions_inner_container)
        self.actions_layout.addStretch()  # Add stretch to push actions to top
        
        # Enable drag and drop for the container
        self.actions_container.setAcceptDrops(True)
        self.actions_container.setStyleSheet("""
            QWidget#actions_container {
                background: #2a2a2a;
                border: 1px dashed #4a4a4a;
                border-radius: 4px;
                min-height: 150px;
                min-width: 300px;
                padding: 5px;
            }
            QScrollArea {
                border: none;
                background: transparent;
            }
            QScrollBar:vertical {
                border: none;
                background: #2a2a2a;
                width: 10px;
                margin: 0px;
            }
            QScrollBar::handle:vertical {
                background: #4a4a4a;
                min-height: 20px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical:hover {
                background: #5a5a5a;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        self.actions_container.setObjectName("actions_container")
        
        actions_scroll.setWidget(self.actions_container)
        actions_layout.addWidget(actions_scroll)
        
        # Add action buttons
        btn_frame = QFrame()
        btn_frame.setFrameShape(QFrame.Shape.StyledPanel)
        btn_layout = QHBoxLayout(btn_frame)
        btn_layout.setContentsMargins(5, 5, 5, 5)
        
        for action_type in ActionType:
            # Skip MOVE_TO as it's redundant with MOVE
            if action_type == ActionType.MOVE_TO:
                continue
                
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
    
    def add_action(self, action_data: dict, index: int = None):
        """Add a new action to the step."""
        action_widget = ActionEditor(action_data, self)
        
        # Make action widget draggable
        action_widget.setAcceptDrops(True)
        action_widget.drag_start_position = None
        
        # Set minimum height for the action editor
        action_widget.setMinimumHeight(60)
        
        # Store reference to parent step editor
        action_widget.step_editor = self
        
        # Add to widgets list
        if index is not None:
            self.action_widgets.insert(index, action_widget)
        else:
            self.action_widgets.append(action_widget)
        
        # Insert before the stretch at the end or at specified index
        insert_pos = index if index is not None else (self.actions_layout.count() - 1)
        self.actions_layout.insertWidget(insert_pos, action_widget)
        
        # Connect remove signal
        action_widget.remove_requested.connect(self.remove_action)
    
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
        
        # Add a small delay to ensure the UI has updated
        try:
            if hasattr(self, 'scroll_area') and self.scroll_area:
                QTimer.singleShot(100, self.scroll_to_bottom)
        except RuntimeError:
            # Widget was already deleted, ignore
            pass
        
        return step_editor
    
    def scroll_to_bottom(self):
        """Scroll the scroll area to the bottom."""
        try:
            # Check if scroll area and its scrollbar still exist
            if (hasattr(self, 'scroll_area') and self.scroll_area and 
                self.scroll_area.verticalScrollBar()):
                scrollbar = self.scroll_area.verticalScrollBar()
                scrollbar.setValue(scrollbar.maximum())
        except RuntimeError:
            # Widget was already deleted, ignore
            pass
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
        """Enable or disable the failsafe UI elements."""
        if hasattr(self, 'failsafe_enable_checkbox'):
            self.failsafe_enable_checkbox.setEnabled(enabled)
        if hasattr(self, 'fs_region_btn'):
            self.fs_region_btn.setEnabled(enabled)
        if hasattr(self, 'fs_template_combo'):
            self.fs_template_combo.setEnabled(enabled)
        if hasattr(self, 'fs_interval_spin'):
            self.fs_interval_spin.setEnabled(enabled)
        if not enabled:
            self.failsafe_region = None
            if hasattr(self, 'fs_region_label'):
                self.fs_region_label.setText("Region: Full Screen")


class RegionSelectionDialog(QDialog):
    """Dialog for selecting a region of the screen for preview."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Preview Region")
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | 
                           Qt.WindowType.WindowStaysOnTopHint |
                           Qt.WindowType.Tool)
        
        # Make window semi-transparent
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setWindowOpacity(0.3)
        
        # Store screen information
        self.screens = QGuiApplication.screens()
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
        
        # Instructions label
        self.instructions = QLabel("Click and drag to select a region. Press Enter to confirm or Esc to cancel.", self)
        self.instructions.setStyleSheet("""
            QLabel {
                background-color: rgba(0, 0, 0, 180);
                color: white;
                padding: 5px;
                border-radius: 5px;
            }
        """)
        self.instructions.adjustSize()
        self.instructions.move(10, 10)
        self.instructions.show()
    
    def paintEvent(self, event):
        """Draw the semi-transparent overlay and selection rectangle."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Draw semi-transparent overlay
        painter.fillRect(self.rect(), QColor(0, 0, 0, 100))
        
        # Draw selection rectangle
        if self.selection_rect:
            painter.setPen(QPen(Qt.GlobalColor.white, 2, Qt.PenStyle.SolidLine))
            painter.drawRect(self.selection_rect)
            
            # Fill the selected area with a different opacity
            painter.fillRect(self.selection_rect, QColor(255, 255, 255, 30))
    
    def mousePressEvent(self, event):
        """Start selection on mouse press."""
        if event.button() == Qt.MouseButton.LeftButton:
            self.start_pos = event.position().toPoint()
            self.end_pos = self.start_pos
            self.selection_rect = QRect()
            self.update()
    
    def mouseMoveEvent(self, event):
        """Update selection rectangle on mouse move."""
        if self.start_pos:
            self.end_pos = event.position().toPoint()
            self.selection_rect = QRect(
                min(self.start_pos.x(), self.end_pos.x()),
                min(self.start_pos.y(), self.end_pos.y()),
                abs(self.end_pos.x() - self.start_pos.x()),
                abs(self.end_pos.y() - self.start_pos.y())
            )
            self.update()
    
    def mouseReleaseEvent(self, event):
        """Finish selection on mouse release."""
        if event.button() == Qt.MouseButton.LeftButton and self.start_pos:
            self.end_pos = event.position().toPoint()
            self.selection_rect = QRect(
                min(self.start_pos.x(), self.end_pos.x()),
                min(self.start_pos.y(), self.end_pos.y()),
                abs(self.end_pos.x() - self.start_pos.x()),
                abs(self.end_pos.y() - self.start_pos.y())
            )
            self.update()
    
    def keyPressEvent(self, event):
        """Handle Enter to confirm or Escape to cancel."""
        if event.key() == Qt.Key.Key_Return or event.key() == Qt.Key.Key_Enter:
            if self.selection_rect and self.selection_rect.isValid():
                self.accept()
            else:
                QMessageBox.warning(self, "Invalid Selection", "Please select a valid region.")
        elif event.key() == Qt.Key.Key_Escape:
            self.reject()
    
    def get_selected_region(self):
        """Get the selected region in screen coordinates."""
        if self.selection_rect and self.selection_rect.isValid():
            # Convert to screen coordinates
            screen_rect = QRect(
                self.combined_geometry.x() + self.selection_rect.x(),
                self.combined_geometry.y() + self.selection_rect.y(),
                self.selection_rect.width(),
                self.selection_rect.height()
            )
            return (
                screen_rect.x(), 
                screen_rect.y(), 
                screen_rect.width(), 
                screen_rect.height()
            )
        return None


class TemplatePreviewWindow(QDialog):
    """A window that displays the current screen with template matches highlighted."""
    def __init__(self, bot, parent=None):
        super().__init__(parent)
        self.bot = bot
        self.setWindowTitle("Template Matching Preview")
        self.setMinimumSize(800, 600)
        self.preview_region = None  # Will store (x, y, width, height)
        self.last_update_time = 0
        self.update_interval = 300  # ms between updates
        
        # Main layout
        layout = QVBoxLayout()
        
        # Controls layout
        controls_layout = QHBoxLayout()
        
        # Template selection
        self.template_combo = QComboBox()
        self.template_combo.currentTextChanged.connect(self.schedule_update_preview)
        controls_layout.addWidget(QLabel("Template:"))
        controls_layout.addWidget(self.template_combo)
        
        # Region selection button
        self.region_btn = QPushButton("Select Region")
        self.region_btn.clicked.connect(self.select_region)
        controls_layout.addWidget(self.region_btn)
        
        # Clear region button
        self.clear_region_btn = QPushButton("Clear Region")
        self.clear_region_btn.clicked.connect(self.clear_region)
        self.clear_region_btn.setEnabled(False)
        controls_layout.addWidget(self.clear_region_btn)
        
        # Confidence threshold
        self.confidence_slider = QSlider(Qt.Orientation.Horizontal)
        self.confidence_slider.setRange(0, 100)
        self.confidence_slider.setValue(int(self.bot.confidence * 100))
        self.confidence_slider.valueChanged.connect(self.on_confidence_changed)
        controls_layout.addWidget(QLabel("Confidence:"))
        controls_layout.addWidget(self.confidence_slider)
        
        # Confidence value label
        self.confidence_label = QLabel(f"{self.bot.confidence:.2f}")
        controls_layout.addWidget(self.confidence_label)
        
        # Add stretch to push everything to the left
        controls_layout.addStretch()
        
        # Add controls to main layout
        layout.addLayout(controls_layout)
        
        # Image display area
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        
        # Add scroll area for the image
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidget(self.image_label)
        self.scroll_area.setWidgetResizable(True)
        layout.addWidget(self.scroll_area)
        
        # Status bar
        self.status_bar = QStatusBar()
        layout.addWidget(self.status_bar)
        
        # Set the main layout
        self.setLayout(layout)
        
        # Load templates
        self.load_templates()
        
        # Set up update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_preview)
        self.update_timer.start(100)  # Check for updates 10 times per second
    
    def select_region(self):
        """Open a dialog to select a region of the screen."""
        dialog = RegionSelectionDialog(self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self.preview_region = dialog.get_selected_region()
            self.clear_region_btn.setEnabled(True)
            self.schedule_update_preview()
    
    def clear_region(self):
        """Clear the selected region."""
        self.preview_region = None
        self.clear_region_btn.setEnabled(False)
        self.schedule_update_preview()
    
    def schedule_update_preview(self):
        """Schedule a preview update if enough time has passed since the last update."""
        current_time = QDateTime.currentMSecsSinceEpoch()
        if current_time - self.last_update_time > self.update_interval:
            self.update_preview()
            self.last_update_time = current_time
    
    def load_templates(self):
        """Load available templates into the combo box."""
        self.template_combo.clear()
        if hasattr(self.parent(), 'config') and 'templates' in self.parent().config:
            self.template_combo.addItems(self.parent().config['templates'].keys())

    def on_confidence_changed(self, value):
        """Handle confidence threshold change."""
        confidence = value / 100.0
        self.confidence_label.setText(f"{confidence:.2f}")
        self.bot.confidence = confidence
        self.schedule_update_preview()

    def update_preview(self):
        """Update the preview with the current template matches."""
        current_time = QDateTime.currentMSecsSinceEpoch()
        if current_time - self.last_update_time < self.update_interval:
            return
        
        self.last_update_time = current_time
            
        if not hasattr(self, 'bot') or not self.template_combo.currentText():
            return
            
        template_name = self.template_combo.currentText()
        
        try:
            # Get the screen capture with matches drawn
            result = self.bot.find_on_screen(
                template_name, 
                region=self.preview_region,
                draw_matches=True
            )
            
            if result is not None and len(result) >= 3:
                _, confidence, screenshot = result
                if screenshot is not None:
                    # Convert numpy array to QImage
                    height, width, channel = screenshot.shape
                    bytes_per_line = 3 * width
                    q_img = QImage(screenshot.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
                    
                    # Create pixmap
                    pixmap = QPixmap.fromImage(q_img)
                    
                    # Only scale down, never up
                    if pixmap.width() > self.scroll_area.viewport().width() or \
                       pixmap.height() > self.scroll_area.viewport().height():
                        pixmap = pixmap.scaled(
                            self.scroll_area.viewport().size(),
                            Qt.AspectRatioMode.KeepAspectRatio,
                            Qt.TransformationMode.SmoothTransformation
                        )
                    
                    self.image_label.setPixmap(pixmap)
                    
                    # Show status with confidence
                    if confidence > 0:
                        self.status_bar.showMessage(
                            f"Template '{template_name}' found with confidence: {confidence:.2f}"
                        )
                    else:
                        self.status_bar.showMessage(f"Template '{template_name}' not found")
                    return
            
            self.status_bar.showMessage(f"Template '{template_name}' not found")
            
        except Exception as e:
            logger.error(f"Error updating preview: {e}")
            self.status_bar.showMessage(f"Error: {str(e)}")

    def showEvent(self, event):
        """Handle the window show event."""
        super().showEvent(event)
        self.load_templates()
        self.schedule_update_preview()

    def closeEvent(self, event):
        """Handle the window close event."""
        self.update_timer.stop()
        event.accept()


class MainWindow(QMainWindow):
    """Main application window."""
    def __init__(self):
        super().__init__()
        self.config = {}
        # Set config path to be in the same directory as the script
        self.config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
        self.bot = ImageDetectionBot()
        self.search_region = None  # Will store the search region (x, y, width, height)
        self.worker = None  # Will store the worker thread
        self.f8_pressed = False  # Track F8 key state
        
        # Initialize images directory
        self.images_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")
        os.makedirs(self.images_dir, exist_ok=True)
        
        # Initialize UI components
        self.templates_list = QListWidget()
        self.sequences_list = QListWidget()
        
        # Set up auto-save timer
        self.auto_save_timer = QTimer(self)
        self.auto_save_timer.timeout.connect(self.auto_save)
        self.auto_save_timer.start(2147483647)  # Auto-save every 30 seconds
        
        self.init_ui()
        self.load_config()  # Load config on startup
        
        # Set up F8 key monitoring
        self.f8_timer = QTimer(self)
        self.f8_timer.timeout.connect(self.check_f8_key)
        self.f8_timer.start(100)  # Check every 100ms

        # Set up mouse position tracking
        self.mouse_pos_label = QLabel()
        self.statusBar().addPermanentWidget(self.mouse_pos_label)
        self.mouse_timer = QTimer(self)
        self.mouse_timer.timeout.connect(self.update_mouse_position)
        self.mouse_timer.start(100)  # Update every 100ms
    
    def init_ui(self):
        # Create main window
        self.setWindowTitle("Bot GUI")
        self.setGeometry(100, 100, 800, 600)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create tab widget
        tab_widget = QTabWidget()
        
        # Create tabs
        sequence_tab = QWidget()
        templates_tab = QWidget()
        settings_tab = QWidget()
        
        # Initialize tab contents
        self.init_sequence_tab(sequence_tab)
        self.init_templates_tab(templates_tab)
        self.init_settings_tab(settings_tab)
        
        # Add tabs to widget
        tab_widget.addTab(sequence_tab, "Sequence")
        tab_widget.addTab(templates_tab, "Templates")
        tab_widget.addTab(settings_tab, "Settings")
        
        main_layout.addWidget(tab_widget)
        
        # Create control buttons
        button_layout = QHBoxLayout()
        
        run_button = QPushButton("Run")
        run_button.clicked.connect(self.run_sequence)
        button_layout.addWidget(run_button)
        
        stop_button = QPushButton("Stop")
        stop_button.clicked.connect(self.stop_sequence)
        stop_button.setEnabled(False)  # Initially disabled
        self.stop_action = stop_button  # Store reference to stop button
        button_layout.addWidget(stop_button)
        
        main_layout.addLayout(button_layout)
        
        # Create status bar
        self.statusBar().showMessage("Ready")
        
        # Set main layout
        self.setLayout(main_layout)
    
    def add_sequence(self):
        """Add a new sequence to the configuration."""
        # Generate a default sequence name
        sequence_count = len(self.config.get('sequences', []))
        default_name = f"Sequence {sequence_count + 1}"
        
        # Get user input for sequence name
        name, ok = QInputDialog.getText(
            self,
            'Add New Sequence',
            'Enter sequence name:',
            text=default_name
        )
        
        if ok and name.strip():
            # Check if name already exists
            existing_sequences = [seq.get('name', '') for seq in self.config.get('sequences', [])]
            if name in existing_sequences:
                QMessageBox.warning(
                    self,
                    'Duplicate Name',
                    f'A sequence named "{name}" already exists. Please choose a different name.',
                    QMessageBox.StandardButton.Ok
                )
                return
                
            # Create new sequence
            new_sequence = {
                'name': name,
                'steps': [],
                'loop': False,
                'loop_count': 1
            }
            
            # Add to config
            if 'sequences' not in self.config:
                self.config['sequences'] = []
            self.config['sequences'].append(new_sequence)
            
            # Update UI
            self.sequences_list.addItem(name)
            self.sequences_list.setCurrentRow(self.sequences_list.count() - 1)
            
            # Save config
            self.save_config()
            self.statusBar().showMessage(f'Added sequence: {name}')
    
    def rename_sequence(self):
        """Rename the currently selected sequence."""
        current_item = self.sequences_list.currentItem()
        if not current_item:
            return
            
        old_name = current_item.text()
        new_name, ok = QInputDialog.getText(
            self, 
            "Rename Sequence", 
            "Enter new sequence name:",
            text=old_name
        )
        
        if ok and new_name and new_name != old_name:
            # Check if name already exists
            if any(s.get('name') == new_name for s in self.config.get('sequences', [])):
                QMessageBox.warning(self, "Error", f"A sequence named '{new_name}' already exists.")
                return
                
            # Update in config
            for sequence in self.config.get('sequences', []):
                if sequence.get('name') == old_name:
                    sequence['name'] = new_name
                    break
            
            # Update UI
            current_item.setText(new_name)
            self.statusBar().showMessage(f"Renamed sequence to '{new_name}'", 3000)
    
    def copy_sequence(self):
        """Create a copy of the currently selected sequence."""
        current_item = self.sequences_list.currentItem()
        if not current_item:
            return
            
        old_name = current_item.text()
        new_name = f"{old_name} (Copy)"
        
        # Find a unique name
        counter = 1
        while any(s.get('name') == new_name for s in self.config.get('sequences', [])):
            new_name = f"{old_name} (Copy {counter})"
            counter += 1
        
        # Find the sequence to copy
        for sequence in self.config.get('sequences', []):
            if sequence.get('name') == old_name:
                # Create a deep copy of the sequence
                import copy
                new_sequence = copy.deepcopy(sequence)
                new_sequence['name'] = new_name
                
                # Add to config
                self.config['sequences'].append(new_sequence)
                
                # Update UI
                self.sequences_list.addItem(new_name)
                self.sequences_list.setCurrentRow(self.sequences_list.count() - 1)
                self.statusBar().showMessage(f"Created copy: {new_name}", 3000)
                break
    
    def remove_sequence(self):
        """Safely remove the currently selected sequence."""
        current_item = self.sequences_list.currentItem()
        if not current_item:
            return
            
        sequence_name = current_item.text()
        
        # Check if sequence is being used in any other sequences
        used_in = []
        for seq in self.config.get('sequences', []):
            if seq.get('name') != sequence_name:  # Don't check against self
                for step in seq.get('steps', []):
                    if step.get('find') == sequence_name:
                        used_in.append(seq['name'])
        
        if used_in:
            reply = QMessageBox.warning(
                self,
                "Sequence in Use",
                f"This sequence is used in: {', '.join(used_in)}\n\n"
                "Deleting it may cause issues. Are you sure you want to continue?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
        
        # Confirm deletion
        reply = QMessageBox.question(
            self,
            "Confirm Delete",
            f"Are you sure you want to delete the sequence '{sequence_name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            # Remove from config
            self.config['sequences'] = [s for s in self.config['sequences'] 
                                      if s.get('name') != sequence_name]
            
            # Update UI
            self.sequences_list.takeItem(self.sequences_list.row(current_item))
            
            # Clear the sequence editor if it was showing this sequence
            if hasattr(self, 'sequence_editor_widget'):
                self.main_content_layout.removeWidget(self.sequence_editor_widget)
                self.sequence_editor_widget.deleteLater()
                del self.sequence_editor_widget
            
            self.statusBar().showMessage(f"Deleted sequence: {sequence_name}", 3000)
    
    def init_sequence_tab(self, tab):
        """Initialize the sequence tab."""
        layout = QHBoxLayout(tab)
        
        # Left sidebar for sequence list
        sidebar = QWidget()
        sidebar.setFixedWidth(200)
        sidebar_layout = QVBoxLayout(sidebar)
        
        # Sequence list
        self.sequences_list = QListWidget()
        self.sequences_list.currentItemChanged.connect(self.on_sequence_selected)
        sidebar_layout.addWidget(QLabel("Sequences:"))
        sidebar_layout.addWidget(self.sequences_list)
        
        # Buttons for managing sequences
        btn_layout = QVBoxLayout()
        
        # Top row - Add and Remove
        top_btn_layout = QHBoxLayout()
        self.add_sequence_btn = QPushButton("Add")
        self.add_sequence_btn.clicked.connect(self.add_sequence)
        self.remove_sequence_btn = QPushButton("Delete")
        self.remove_sequence_btn.clicked.connect(self.remove_sequence)
        self.remove_sequence_btn.setEnabled(False)
        top_btn_layout.addWidget(self.add_sequence_btn)
        top_btn_layout.addWidget(self.remove_sequence_btn)
        
        # Bottom row - Rename and Copy
        bottom_btn_layout = QHBoxLayout()
        self.rename_sequence_btn = QPushButton("Rename")
        self.rename_sequence_btn.clicked.connect(self.rename_sequence)
        self.rename_sequence_btn.setEnabled(False)
        self.copy_sequence_btn = QPushButton("Copy")
        self.copy_sequence_btn.clicked.connect(self.copy_sequence)
        self.copy_sequence_btn.setEnabled(False)
        bottom_btn_layout.addWidget(self.rename_sequence_btn)
        bottom_btn_layout.addWidget(self.copy_sequence_btn)
        
        # Add all to main button layout
        btn_layout.addLayout(top_btn_layout)
        btn_layout.addLayout(bottom_btn_layout)
        sidebar_layout.addLayout(btn_layout)
        
        # Connect sequence selection change
        self.sequences_list.currentItemChanged.connect(self.on_sequence_selection_changed)
        
        # Loop controls
        loop_group = QGroupBox("Loop Settings")
        loop_layout = QVBoxLayout()
        self.loop_checkbox = QCheckBox("Enable Loop")
        self.loop_checkbox.toggled.connect(self.on_loop_toggled)
        loop_layout.addWidget(self.loop_checkbox)
        
        loop_count_layout = QHBoxLayout()
        self.loop_count_label = QLabel("Loop Count:")
        self.loop_count_spin = QSpinBox()
        self.loop_count_spin.setRange(1, 9999)
        self.loop_count_spin.setValue(1)
        self.loop_count_spin.setEnabled(False)
        loop_count_layout.addWidget(self.loop_count_label)
        loop_count_layout.addWidget(self.loop_count_spin)
        loop_layout.addLayout(loop_count_layout)
        
        loop_group.setLayout(loop_layout)
        sidebar_layout.addWidget(loop_group)
        
        # Add stretch to push everything to the top
        sidebar_layout.addStretch()
        
        # Main content area
        self.main_content = QWidget()
        self.main_content_layout = QVBoxLayout(self.main_content)
        
        # Add widgets to main layout
        layout.addWidget(sidebar)
        layout.addWidget(self.main_content, 1)  # 1 is stretch factor
        
        # Load existing sequences
        self.update_ui_from_config()
    
    def show_template_preview(self):
        """Show the template matching preview window."""
        if not hasattr(self, 'preview_window') or not self.preview_window:
            self.preview_window = TemplatePreviewWindow(self.bot, self)
        self.preview_window.show()
        self.preview_window.raise_()
        self.preview_window.activateWindow()
    
    def init_templates_tab(self, tab):
        """Initialize the templates tab."""
        layout = QVBoxLayout(tab)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Left panel - template list and controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        # Template list
        left_layout.addWidget(QLabel("Templates:"))
        self.templates_list = QListWidget()
        self.templates_list.currentItemChanged.connect(self.on_template_selected)
        left_layout.addWidget(self.templates_list)
        
        # Template buttons
        btn_layout = QHBoxLayout()
        self.add_template_btn = QPushButton("Add")
        self.add_template_btn.clicked.connect(lambda: self.add_template())
        self.delete_template_btn = QPushButton("Delete")
        self.delete_template_btn.clicked.connect(self.delete_template)
        self.delete_template_btn.setEnabled(False)
        btn_layout.addWidget(self.add_template_btn)
        btn_layout.addWidget(self.delete_template_btn)
        left_layout.addLayout(btn_layout)
        
        # Preview button
        self.preview_btn = QPushButton("Preview Matches")
        self.preview_btn.clicked.connect(self.show_template_preview)
        left_layout.addWidget(self.preview_btn)
        
        # Right panel - content area
        self.content_stack = QStackedWidget()
        
        # Add panels to splitter
        splitter.addWidget(left_panel)
        splitter.addWidget(self.content_stack)
        
        # Set initial sizes (left panel: 200px, right panel: takes remaining space)
        splitter.setSizes([200, 600])
        
        # Add splitter to main layout
        layout.addWidget(splitter)
        
        # Load existing templates
        if hasattr(self, 'config') and 'templates' in self.config:
            self.templates_list.addItems(self.config['templates'].keys())
            if self.templates_list.count() > 0:
                self.templates_list.setCurrentRow(0)
    
    def init_settings_tab(self, tab):
        """Initialize the settings tab."""
        layout = QVBoxLayout(tab)
        
        # Create group box for general settings
        general_group = QGroupBox("General Settings")
        general_layout = QFormLayout()
        
        # Search region selection
        region_layout = QHBoxLayout()
        self.region_status = QLabel("Region: Full Screen")
        region_btn = QPushButton("Select Region")
        region_btn.clicked.connect(self.toggle_region_selection)
        region_layout.addWidget(self.region_status)
        region_layout.addWidget(region_btn)
        general_layout.addRow("Search Region:", region_layout)
        
        # Add group box to layout
        general_group.setLayout(general_layout)
        layout.addWidget(general_group)
        
        # Create group box for configuration
        config_group = QGroupBox("Configuration")
        config_layout = QHBoxLayout()
        
        # Configuration buttons
        new_btn = QPushButton("New")
        new_btn.clicked.connect(self.new_config)
        open_btn = QPushButton("Open")
        open_btn.clicked.connect(self.open_config)
        save_btn = QPushButton("Save")
        save_btn.clicked.connect(self.save_config)
        save_as_btn = QPushButton("Save As")
        save_as_btn.clicked.connect(self.save_config_as)
        
        config_layout.addWidget(new_btn)
        config_layout.addWidget(open_btn)
        config_layout.addWidget(save_btn)
        config_layout.addWidget(save_as_btn)
        
        # Add group box to layout
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        # Add stretch to push everything to the top
        layout.addStretch()
    
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
                # Create default config if it doesn't exist
                self.config = {
                    "templates": {},
                    "sequences": []
                }
                self.save_config()
                self.statusBar().showMessage("Created new configuration")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load configuration: {str(e)}")
            # Create default config on error
            self.config = {
                "templates": {},
                "sequences": []
            }
            self.save_config()
    
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
                
            logger.debug(f"Configuration saved to {self.config_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to save configuration: {str(e)}")
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
                    "Error",
                    f"Failed to delete template: {str(e)}",
                    QMessageBox.StandardButton.Ok
                )
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
                if hasattr(self, 'current_sequence_widget'):
                    self.current_sequence_widget.deleteLater()
                    del self.current_sequence_widget
                if hasattr(self, 'sequence_editor_widget'):
                    del self.sequence_editor_widget
                
                # Save the configuration
                self.save_config()
                self.statusBar().showMessage(f"Template '{template_name}' deleted")
                
                # Update the preview window if it exists
                if hasattr(self, 'preview_window') and self.preview_window:
                    self.preview_window.load_templates()
                    
            except Exception as e:
                logger.error(f"Error deleting template: {e}")
                QMessageBox.critical(
                    self,
                    "Error",
                    f"Failed to delete template: {str(e)}",
                    QMessageBox.StandardButton.Ok
                )
    
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
    
    def on_sequence_selection_changed(self, current, previous):
        """Handle sequence selection change to update button states."""
        has_selection = current is not None
        self.remove_sequence_btn.setEnabled(has_selection)
        self.rename_sequence_btn.setEnabled(has_selection)
        self.copy_sequence_btn.setEnabled(has_selection)
        
        # Call the original selection handler
        self.on_sequence_selected(current, previous)
    
    def on_sequence_selected(self, current, previous):
        """Handle sequence selection change."""
        
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
            
            # Clear existing sequence editor if it exists
            if hasattr(self, 'sequence_editor_widget'):
                # Remove the old widget from the layout
                self.main_content_layout.removeWidget(self.sequence_editor_widget)
                # Delete the old widget
                self.sequence_editor_widget.deleteLater()
                # Clear the reference
                del self.sequence_editor_widget
            
            # Create new sequence editor widget
            self.sequence_editor_widget = SequenceEditor(
                sequence, 
                list(self.config.get('templates', {}).keys()), 
                self
            )
            
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
        
        # Stop the auto-save timer
        if hasattr(self, 'auto_save_timer') and self.auto_save_timer.isActive():
            self.auto_save_timer.stop()
        
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
        
        # Final save before closing
        try:
            if hasattr(self, 'sequence_editor_widget'):
                self.save_current_sequence()
            if hasattr(self, 'template_editor'):
                self.save_current_template()
            self.save_config()
        except Exception as e:
            logger.error(f"Error during final save: {e}")
        
        event.accept()

    def auto_save(self):
        """Auto-save the current configuration."""
        try:
            if hasattr(self, 'sequence_editor_widget'):
                self.save_current_sequence()
            if hasattr(self, 'template_editor'):
                self.save_current_template()
            self.save_config()
            logger.debug(f"Auto-saved configuration to {self.config_path}")
        except Exception as e:
            logger.error(f"Error during auto-save: {e}")

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

    # Apply compact global stylesheet for smaller UI
    app.setStyleSheet(app.styleSheet() + """
        QWidget, QMenuBar, QMenu, QToolBar, QStatusBar {
            font-size: 10px;
        }
        QPushButton, QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox, QListWidget, QCheckBox, QLabel {
            min-height: 18px;
            font-size: 10px;
            padding: 1px 4px;
        }
        QGroupBox {
            margin-top: 4px;
            padding: 4px;
            font-size: 10px;
        }
        QTabWidget::pane, QTabBar::tab {
            font-size: 10px;
            min-height: 18px;
        }
        QScrollBar:vertical, QScrollBar:horizontal {
            width: 8px;
            height: 8px;
        }
        QHeaderView::section {
            font-size: 10px;
            padding: 2px;
        }
        QTableWidget, QTableWidgetItem {
            font-size: 10px;
        }
    """)
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    # Run application
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
