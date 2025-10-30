import cv2
import numpy as np
import pyautogui
import time
import os
import json
import sys
from typing import Optional, Tuple, List, Dict, Any, Union
import argparse
import logging
from enum import Enum
from dataclasses import dataclass, asdict
import random
import math

# Configure logging
logging.basicConfig(
    level=logging.DEBUG, 
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('bot_debug.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class ActionType(Enum):
    CLICK = "click"
    MOVE = "move"  # Move to absolute coordinates
    MOVE_TO = "move_to"  # Move to detected template position
    RIGHT_CLICK = "right_click"
    DOUBLE_CLICK = "double_click"
    TYPE = "type"
    KEY_PRESS = "key_press"
    WAIT = "wait"
    SCROLL = "scroll"
    CLICK_AND_HOLD = "click_and_hold"

@dataclass
class Action:
    """Represents a single action to be performed."""
    type: ActionType
    x: Optional[int] = None
    y: Optional[int] = None
    button: str = 'left'
    clicks: int = 1
    text: Optional[str] = None
    key: Optional[str] = None
    seconds: float = 1.0
    pixels: int = 0
    duration: float = 0.0
    region: Optional[List[int]] = None
    random: bool = False
    random_region: Optional[List[int]] = None
    duration: float = 0.0

class ImageDetectionBot:
    def __init__(self, confidence: float = 0.8):
        """
        Initialize the Image Detection Bot.
        
        Args:
            confidence: Confidence threshold for template matching (0.0 to 1.0)
        """
        self.confidence = confidence
        self.templates: Dict[str, np.ndarray] = {}
        self.current_position: Optional[Tuple[int, int]] = None
        self.use_curved_movement = False  # Add default value for movement type
        # Dummy stop event that accepts any arguments but always returns False
        self.stop_event = type('StopEvent', (), {'is_set': lambda *args, **kwargs: False})()
        
    def load_template(self, name: str, image_path: str) -> bool:
        """
        Load an image template from file.
        
        Args:
            name: Name to reference this template
            image_path: Path to the template image file
            
        Returns:
            bool: True if template was loaded successfully, False otherwise
        """
        logger.info(f"Attempting to load template: name='{name}', path='{image_path}'")
        
        # Check if path exists and is a file
        if not os.path.isfile(image_path):
            logger.error(f"Template file does not exist or is not a file: {image_path}")
            return False
            
        # Check file size to ensure it's not empty
        file_size = os.path.getsize(image_path)
        if file_size == 0:
            logger.error(f"Template file is empty: {image_path}")
            return False
            
        logger.info(f"Template file exists, size: {file_size} bytes")
        
        try:
            # Try to read the image
            template = cv2.imread(image_path, cv2.IMREAD_COLOR)
            if template is None:
                error_msg = f"OpenCV failed to load the image. The file might be corrupted or in an unsupported format: {image_path}"
                logger.error(error_msg)
                return False
                
            # Check if image was loaded correctly
            if template.size == 0:
                error_msg = f"Loaded image is empty: {image_path}"
                logger.error(error_msg)
                return False
                
            # Store the template
            self.templates[name] = template
            logger.info(f"Successfully loaded template '{name}' from {image_path}, dimensions: {template.shape[1]}x{template.shape[0]}")
            return True
            
        except Exception as e:
            error_msg = f"Unexpected error loading template '{name}' from {image_path}: {str(e)}"
            logger.error(error_msg, exc_info=True)
            return False
    
    def find_on_screen(self, template_name: str, region: Optional[Tuple[int, int, int, int]] = None, 
                      draw_matches: bool = False) -> Tuple[Optional[Tuple[int, int]], float, Optional[np.ndarray]]:
        """
        Find a template on the screen.
        
        Args:
            template_name: Name of the loaded template to find
            region: Optional (x, y, width, height) region to search in
            draw_matches: If True, returns the screenshot with matches drawn
            
        Returns:
            Tuple containing:
                - Tuple[int, int] or None: (x, y) coordinates of the center of the found image, or None if not found
                - float: Confidence score (0.0 to 1.0)
                - np.ndarray or None: Screenshot with matches drawn (if draw_matches=True)
        """
        logger.debug(f"Searching for template: '{template_name}' in region: {region}")
        
        if template_name not in self.templates:
            available_templates = list(self.templates.keys())
            logger.error(f"Template '{template_name}' not found. Available templates: {available_templates}")
            return None, 0, None
            
        template = self.templates[template_name]
        logger.debug(f"Template dimensions: {template.shape[1]}x{template.shape[0]}")
            
        try:
            # Take screenshot of the specified region or full screen
            screenshot = pyautogui.screenshot(region=region)
            screenshot_np = np.array(screenshot)
            screenshot_bgr = cv2.cvtColor(screenshot_np, cv2.COLOR_RGB2BGR)
            
            # Convert both images to grayscale for matching
            gray_screenshot = cv2.cvtColor(screenshot_bgr, cv2.COLOR_BGR2GRAY)
            gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            
            # Try multiple template matching methods
            methods = [
                (cv2.TM_CCOEFF_NORMED, "CCOEFF_NORMED"),  # Best for most cases
                (cv2.TM_CCORR_NORMED, "CCORR_NORMED"),    # Good for some cases where CCOEFF fails
            ]
            
            best_confidence = 0
            best_method = ""
            best_loc = None
            
            for method, method_name in methods:
                # Perform template matching
                result = cv2.matchTemplate(gray_screenshot, gray_template, method)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                
                # For TM_SQDIFF and TM_SQDIFF_NORMED, the best matches are lower values
                if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                    confidence = 1 - min_val  # Convert to similarity score
                    loc = min_loc
                else:
                    confidence = max_val
                    loc = max_loc
                
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_loc = loc
                    best_method = method_name
            
            if best_confidence >= self.confidence:
                # Calculate center coordinates
                h, w = gray_template.shape[:2]
                x = best_loc[0] + w // 2
                y = best_loc[1] + h // 2
                
                if region:
                    x += region[0]
                    y += region[1]
                
                logger.debug(f"Found '{template_name}' at ({x}, {y}) with confidence {best_confidence:.4f} using {best_method}")
                
                if draw_matches:
                    # Draw the match on the screenshot
                    top_left = best_loc
                    bottom_right = (top_left[0] + w, top_left[1] + h)
                    
                    # Draw rectangle around the match
                    cv2.rectangle(screenshot_bgr, top_left, bottom_right, (0, 255, 0), 2)
                    
                    # Add text with confidence score
                    text = f"{template_name}: {best_confidence:.2f}"
                    cv2.putText(screenshot_bgr, text, 
                              (top_left[0], top_left[1] - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Convert back to RGB for display
                    screenshot_rgb = cv2.cvtColor(screenshot_bgr, cv2.COLOR_BGR2RGB)
                    return (x, y), best_confidence, screenshot_rgb
                
                return (x, y), best_confidence, None
            else:
                logger.debug(f"Could not find '{template_name}' (best match confidence: {best_confidence:.4f} using {best_method})")
                if draw_matches:
                    # Return the screenshot even if no match was found
                    return None, 0, cv2.cvtColor(screenshot_bgr, cv2.COLOR_BGR2RGB)
                return None, 0, None
                
        except Exception as e:
            logger.error(f"Error in find_on_screen: {str(e)}")
            if draw_matches:
                # Return a black image if there was an error
                error_img = np.zeros((100, 100, 3), dtype=np.uint8)
                cv2.putText(error_img, "Error", (10, 50), 
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                return None, 0, error_img
            return None, 0, None
    
    def find_image(self, template_name: str, region: Optional[Tuple[int, int, int, int]] = None, 
                   timeout: float = 10.0, check_interval: float = 0.5, 
                   confidence: Optional[float] = None) -> Optional[Tuple[int, int]]:
        """
        Find a template image on the screen within a specified region.
        
        Args:
            template_name: Name of the template to find
            region: Optional (x, y, width, height) region to search in
            timeout: Maximum time in seconds to search for the image
            check_interval: Time in seconds between search attempts
            confidence: Optional confidence threshold (0.0 to 1.0)
            
        Returns:
            Optional[Tuple[int, int]]: (x, y) coordinates of the center of the found image, or None if not found
        """
        start_time = time.time()
        original_confidence = self.confidence
        
        try:
            # Use the provided confidence if specified, otherwise use the instance confidence
            if confidence is not None:
                self.confidence = confidence
            
            while time.time() - start_time < timeout:
                # Check if we should stop
                if self.stop_event.is_set():
                    logger.debug("Image search interrupted by stop event")
                    return None
                
                # Try to find the image
                result = self.find_on_screen(template_name, region, draw_matches=False)
                if result and result[0] is not None:
                    return result[0]  # Return just the (x, y) position
                
                # Wait before trying again
                time.sleep(check_interval)
                
            logger.debug(f"Could not find '{template_name}' after {timeout} seconds")
            return None
        
        except Exception as e:
            logger.error(f"Error finding image '{template_name}': {str(e)}")
            return None
            
    def type_text(self, text: str) -> None:
        """Type the specified text."""
        try:
            pyautogui.write(text, interval=0.1)
            logger.info(f"Typed: {text}")
        except Exception as e:
            logger.error(f"Error typing text: {str(e)}")
            
    def press_key(self, key: str) -> None:
        """Press the specified key."""
        try:
            pyautogui.press(key)
            logger.info(f"Pressed key: {key}")
        except Exception as e:
            logger.error(f"Error pressing key {key}: {str(e)}")
            
    def move_to(self, x: int, y: int, duration: float = 0.0, random_region: Optional[Tuple[int, int, int, int]] = None) -> bool:
        """
        Move the mouse to the specified coordinates.
        
        Args:
            x: Target x-coordinate
            y: Target y-coordinate
            duration: Time in seconds for the movement
            random_region: Optional (x, y, width, height) region for random movement
            
        Returns:
            bool: True if movement was successful, False otherwise
        """
        try:
            # Handle random region movement if specified
            if random_region and len(random_region) == 4:
                try:
                    rx, ry, rw, rh = random_region
                    # Ensure the random point is within the region
                    target_x = random.randint(rx, rx + rw - 1)
                    target_y = random.randint(ry, ry + rh - 1)
                    logger.info(f"Moving to random position in region {random_region}: ({target_x}, {target_y})")
                except Exception as e:
                    logger.error(f"Error calculating random position: {str(e)}")
                    return False
            else:
                target_x, target_y = x, y
            
            # Ensure coordinates are within screen bounds
            screen_width, screen_height = pyautogui.size()
            target_x = max(0, min(target_x, screen_width - 1))
            target_y = max(0, min(target_y, screen_height - 1))
            
            # Perform the movement
            if duration > 0:
                # Use curved movement if enabled
                if getattr(self, 'use_curved_movement', False):
                    # Get current position
                    current_x, current_y = pyautogui.position()
                    
                    # Generate control points for curve
                    control_x = current_x + (target_x - current_x) * 0.5 + random.uniform(-100, 100)
                    control_y = current_y + (target_y - current_y) * 0.5 + random.uniform(-50, 50)
                    
                    # Generate points along the curve
                    steps = max(5, int(duration * 10))  # At least 5 steps
                    points = []
                    for i in range(steps + 1):
                        t = i / steps
                        # Quadratic bezier curve
                        x = (1-t)**2 * current_x + 2*(1-t)*t*control_x + t**2 * target_x
                        y = (1-t)**2 * current_y + 2*(1-t)*t*control_y + t**2 * target_y
                        points.append((int(x), int(y)))
                    
                    # Move through the points
                    for point in points:
                        pyautogui.moveTo(*point)
                        time.sleep(duration / steps)
                else:
                    # Linear movement
                    pyautogui.moveTo(target_x, target_y, duration=duration)
            else:
                # Instant movement
                pyautogui.moveTo(target_x, target_y)
            
            # Update current position
            self.current_position = (target_x, target_y)
            logger.debug(f"Moved to ({target_x}, {target_y}){f' over {duration:.2f}s' if duration > 0 else ''}")
            return True
            
        except Exception as e:
            logger.error(f"Error in move_to: {str(e)}")
            return False
            
    def click_at(self, x: int, y: int, button: str = 'left', clicks: int = 1, interval: float = 0.1) -> bool:
        """
        Click at the specified coordinates.
        
        Args:
            x: X coordinate to click
            y: Y coordinate to click
            button: Mouse button to click ('left', 'right', 'middle')
            clicks: Number of clicks to perform
            interval: Time in seconds between clicks if multiple clicks
            
        Returns:
            bool: True if click was successful, False otherwise
        """
        try:
            # Validate button
            button = button.lower()
            if button not in ('left', 'right', 'middle'):
                logger.warning(f"Invalid button '{button}'. Using 'left' instead.")
                button = 'left'
                
            # Ensure clicks is at least 1
            clicks = max(1, int(clicks))
            
            # Move to the position first
            if not self.move_to(x, y, duration=0.1):
                logger.error(f"Failed to move to click position: ({x}, {y})")
                return False
                
            # Small delay before clicking
            time.sleep(0.1)
            
            # Perform the click(s)
            for i in range(clicks):
                pyautogui.click(button=button)
                if i < clicks - 1:  # Don't wait after the last click
                    time.sleep(interval)
            
            logger.debug(f"Clicked at ({x}, {y}) with {button} button ({clicks} clicks)")
            return True
            
        except Exception as e:
            logger.error(f"Error in click_at: {str(e)}")
            return False
    
    def wait(self, seconds: float) -> None:
        """Wait for the specified number of seconds."""
        time.sleep(seconds)
        logger.info(f"Waited for {seconds} seconds")
    
    def wait_for_image(self, template_name: str, timeout: float = 10.0, interval: float = 0.5,
                      confidence: Optional[float] = None) -> Optional[Tuple[int, int]]:
        """
        Wait for an image to appear on screen.
        
        Args:
            template_name: Name of the template to wait for
            timeout: Maximum time to wait in seconds
            interval: Time between checks in seconds
            confidence: Confidence threshold (0.0 to 1.0). If None, uses the bot's default confidence.
            
        Returns:
            Optional[Tuple[int, int]]: Coordinates where the image was found, or None if not found
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            position = self.find_image(template_name, confidence=confidence)
            if position is not None:
                return position
            time.sleep(interval)
            
        logger.warning(f"Timed out waiting for '{template_name}'")
        return None
        
    def execute_action_at_position(self, action: Action, position: Optional[Tuple[int, int]] = None) -> bool:
        """
        Execute an action at a specific position.
        
        Args:
            action: Action to execute
            position: Optional position to execute the action at. If None, uses the bot's current position.
            
        Returns:
            bool: True if the action was executed successfully, False otherwise
        """
        if position is None:
            if self.current_position is None:
                logger.error("No position provided and no current position set")
                return False
            position = self.current_position
            
        x, y = position
        
        # If this is a click action and a region is specified, pick a random point in the region
        if action.type == ActionType.CLICK and hasattr(action, 'region') and action.region:
            region = action.region
            if isinstance(region, (list, tuple)) and len(region) == 4:
                x = random.randint(region[0], region[0] + region[2] - 1)
                y = random.randint(region[1], region[1] + region[3] - 1)
                try:
                    self.click_at(x, y, button=action.button, clicks=action.clicks)
                    return True
                except Exception as e:
                    logger.error(f"Error executing random region click: {str(e)}")
                    return False
        # If this is a move_to action and random is set, move to a random point in the region
        if action.type == ActionType.MOVE_TO and getattr(action, 'random', False) and hasattr(action, 'random_region') and action.random_region:
            region = action.random_region
            if isinstance(region, (list, tuple)) and len(region) == 4:
                x = random.randint(region[0], region[0] + region[2] - 1)
                y = random.randint(region[1], region[1] + region[3] - 1)
                try:
                    self.move_to(x, y, duration=action.duration)
                    return True
                except Exception as e:
                    logger.error(f"Error executing random region move_to: {str(e)}")
                    return False
        
        try:
            if action.type == ActionType.CLICK:
                # Ensure button attribute exists and is valid
                button = getattr(action, 'button', 'left')
                clicks = getattr(action, 'clicks', 1)
                self.click_at(x, y, button=button, clicks=clicks)
                
            elif action.type == ActionType.MOVE:
                self.move_to(x, y, duration=getattr(action, 'duration', 0.0))
                
            elif action.type == ActionType.MOVE_TO:
                self.move_to(x, y, duration=getattr(action, 'duration', 0.0))
                
            elif action.type == ActionType.RIGHT_CLICK:
                self.right_click_at(x, y)
                
            elif action.type == ActionType.DOUBLE_CLICK:
                self.double_click_at(x, y)
                
            elif action.type == ActionType.TYPE:
                text = getattr(action, 'text', '')
                if text:
                    self.type_text(text)
                else:
                    logger.warning("No text provided for TYPE action")
                    return False
                    
            elif action.type == ActionType.KEY_PRESS:
                key = getattr(action, 'key', None)
                if key:
                    self.press_key(key)
                else:
                    logger.warning("No key provided for KEY_PRESS action")
                    return False
                    
            elif action.type == ActionType.WAIT:
                seconds = getattr(action, 'seconds', 1.0)
                self.wait(seconds)
                
            elif action.type == ActionType.SCROLL:
                pixels = getattr(action, 'pixels', 0)
                self.scroll(pixels)
                
            elif action.type == ActionType.CLICK_AND_HOLD:
                try:
                    if self.current_position is None:
                        logger.error("No current position set for CLICK_AND_HOLD")
                        return False
                        
                    x, y = self.current_position
                    duration = getattr(action, 'duration', 1.0)
                    button = getattr(action, 'button', 'left')
                    
                    logger.info(f"Executing CLICK_AND_HOLD at current position: ({x}, {y}) for {duration} seconds")
                    
                    # Move to the position first
                    self.move_to(x, y, duration=0.1)
                    
                    # Perform the click and hold
                    pyautogui.mouseDown(button=button)
                    time.sleep(duration)
                    pyautogui.mouseUp(button=button)
                    
                    return True
                except Exception as e:
                    logger.error(f"Error in CLICK_AND_HOLD: {str(e)}")
                    return False
            else:
                logger.error(f"Unsupported action type: {action.type}")
                return False
                
            return True
            
        except Exception as e:
            logger.error(f"Error executing action {action.type.value}: {str(e)}")
            return False
            
    def execute_action(self, action: Action) -> bool:
        """
        Execute a single action.
        
        Args:
            action: Action to execute
            
        Returns:
            bool: True if action was executed successfully, False otherwise
        """
        try:
            if action.type == ActionType.MOVE and action.x is not None and action.y is not None:
                # For MOVE action with absolute coordinates
                if getattr(action, 'random', False) and hasattr(action, 'random_region') and action.random_region:
                    region = action.random_region
                    if isinstance(region, (list, tuple)) and len(region) == 4:
                        x = random.randint(region[0], region[0] + region[2] - 1)
                        y = random.randint(region[1], region[1] + region[3] - 1)
                        pyautogui.moveTo(x, y, duration=action.duration)
                        return True
                else:
                    pyautogui.moveTo(action.x, action.y, duration=action.duration)
                    return True
            elif action.type == ActionType.CLICK_AND_HOLD:
                # Allow click_and_hold to be executed directly, using current mouse position
                return self.execute_action_at_position(action, self.current_position)
            else:
                # For other actions, use execute_action_at_position with current position
                return self.execute_action_at_position(action, self.current_position)
        except Exception as e:
            logger.error(f"Error executing {action.type.value if hasattr(action.type, 'value') else action.type} action: {str(e)}")
            return False
            
    def execute_actions(self, actions: List[Action]) -> bool:
        """
        Execute a list of actions.
        
        Args:
            actions: List of actions to execute
            
        Returns:
            bool: True if all actions were executed successfully, False otherwise
        """
        for action in actions:
            if not self.execute_action(action):
                return False
        return True
    
    def scroll(self, pixels: int) -> None:
        """Scroll the mouse wheel.
        
        Args:
            pixels: Number of pixels to scroll (positive for up, negative for down)
        """
        try:
            pyautogui.scroll(pixels)
            logger.info(f"Scrolled {pixels} pixels")
        except Exception as e:
            logger.error(f"Error scrolling: {str(e)}")

def load_config(config_path: str) -> dict:
    """Load configuration from JSON file."""
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Error loading config: {str(e)}")
        return {}

def parse_action(action_dict: Dict[str, Any]) -> Action:
    """Parse action dictionary into Action object."""
    try:
        action_type = ActionType(action_dict['type'])
        action_args = {k: v for k, v in action_dict.items() if k != 'type'}
        return Action(type=action_type, **action_args)
    except (KeyError, ValueError) as e:
        logger.error(f"Error parsing action: {str(e)}")
        raise

def execute_sequence(bot: ImageDetectionBot, sequence: Dict[str, Any]) -> bool:
    """
    Execute a sequence of steps with actions.
    
    Args:
        bot: Initialized ImageDetectionBot instance
        sequence: Dictionary containing sequence configuration
        
    Returns:
        bool: True if sequence completed successfully, False otherwise
    """
    try:
        steps = sequence.get('steps', [])
        sequence_name = sequence.get('name', 'unnamed')
        if not steps:
            logger.error("No steps found in sequence")
            return False
            
        for step in steps:
            template_name = step.get('find')
            required = step.get('required', True)
            timeout = step.get('timeout', 10.0)
            confidence = step.get('confidence')
            
            if not template_name:
                # If no template to find, just execute actions
                actions = [parse_action(a) for a in step.get('actions', [])]
                if not bot.execute_actions(actions):
                    if required:
                        return False
                    continue
                
                continue
                
            # Find the template
            logger.info(f"Looking for template: {template_name}")
            position = bot.find_image(template_name, timeout=timeout, confidence=confidence)
            
            if position is None:
                if required:
                    logger.error(f"Required template not found: {template_name}")
                    return False
                continue
                
            # Execute actions for this step
            actions = [parse_action(a) for a in step.get('actions', [])]
            
            # Track the last position from MOVE commands
            last_move_position = None
            
            for action in actions:
                # Handle MOVE action first to update the last position
                if action.type == ActionType.MOVE:
                    if action.x is not None and action.y is not None:
                        bot.move_to(action.x, action.y, duration=action.duration)
                        last_move_position = (action.x, action.y)
                        position = last_move_position  # Keep this for backward compatibility
                        continue
                
                # For CLICK_AND_HOLD, use the last moved position
                if action.type == ActionType.CLICK_AND_HOLD:
                    try:
                        if last_move_position is None:
                            error_msg = "No previous MOVE command found for CLICK_AND_HOLD"
                            logger.error(error_msg)
                            if required:
                                return False
                            break
                        
                        # Update the bot's current position to the last moved position
                        bot.current_position = last_move_position
                        
                        # Execute the CLICK_AND_HOLD action
                        if not bot.execute_action(action):
                            if required:
                                return False
                            break
                            
                        continue
                    except Exception as e:
                        logger.error(f"Error in CLICK_AND_HOLD: {str(e)}")
                        if required:
                            return False
                        break
                
                # For all other actions
                if not bot.execute_action_at_position(action, position):
                    if required:
                        return False
                    break
        
        logger.info(f"Completed sequence: {sequence_name}")
        return True
        
    except Exception as e:
        logger.error(f"Error executing sequence '{sequence_name}': {str(e)}")
        return False

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Image Detection Bot')
    parser.add_argument('--templates', nargs='+', help='Template image files (format: name=path)')
    parser.add_argument('--find', help='Name of the template to find')
    parser.add_argument('--click', action='store_true', help='Click on the found image')
    parser.add_argument('--wait', action='store_true', help='Wait for the image to appear')
    parser.add_argument('--timeout', type=float, default=10.0, help='Timeout in seconds (default: 10)')
    parser.add_argument('--confidence', type=float, default=0.8, help='Confidence threshold (0.0 to 1.0, default: 0.8)')
    parser.add_argument('--config', help='Path to config JSON file')
    parser.add_argument('--sequence', help='Name of the sequence to run from config')
    args = parser.parse_args()
    
    # Initialize the bot
    bot = ImageDetectionBot(confidence=args.confidence)
    
    # Load from config file if provided
    config = {}
    if args.config:
        config = load_config(args.config)
        
        # Load templates from config
        for name, path in config.get('templates', {}).items():
            if not bot.load_template(name, path):
                logger.warning(f"Failed to load template '{name}' from config")
    
    # Load templates from command line
    if args.templates:
        for template in args.templates:
            if '=' in template:
                name, path = template.split('=', 1)
                if not bot.load_template(name.strip(), path.strip()):
                    logger.warning(f"Failed to load template '{name}' from command line")
    
    # Run sequence from config if specified
    if args.sequence and config:
        sequence = next((s for s in config.get('sequences', []) if s.get('name') == args.sequence), None)
        if sequence:
            success = execute_sequence(bot, sequence)
            return 0 if success else 1
        else:
            logger.error(f"Sequence '{args.sequence}' not found in config")
            return 1
    
    # Legacy single-template mode
    if args.find:
        if args.wait:
            position = bot.wait_for_image(args.find, timeout=args.timeout)
        else:
            position = bot.find_on_screen(args.find)
            
        if position is not None:
            print(f"Found '{args.find}' at {position}")
            if args.click:
                bot.click_at(*position)
        else:
            print(f"Could not find '{args.find}'")
            return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
