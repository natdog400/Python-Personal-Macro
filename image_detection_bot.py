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
    type: ActionType
    button: str = "left"
    clicks: int = 1
    text: Optional[str] = None
    key: Optional[str] = None
    seconds: float = 1.0
    pixels: int = 0
    x: Optional[int] = None
    y: Optional[int] = None
    duration: float = 0.0
    region: Optional[Tuple[int, int, int, int]] = None
    random: bool = False
    random_region: Optional[Tuple[int, int, int, int]] = None

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
    
    def find_on_screen(self, template_name: str, region: Optional[Tuple[int, int, int, int]] = None) -> Optional[Tuple[int, int]]:
        """
        Find a template on the screen.
        
        Args:
            template_name: Name of the loaded template to find
            region: Optional (x, y, width, height) region to search in
            
        Returns:
            Optional[Tuple[int, int]]: (x, y) coordinates of the center of the found image, or None if not found
        """
        logger.debug(f"Searching for template: '{template_name}' in region: {region}")
        
        if template_name not in self.templates:
            # Log available templates for debugging
            available_templates = list(self.templates.keys())
            logger.error(
                f"Template '{template_name}' not found. Available templates: {available_templates}"
            )
            return None
            
        template = self.templates[template_name]
        logger.debug(f"Template dimensions: {template.shape[1]}x{template.shape[0]}")
            
        try:
            # Take screenshot of the specified region or full screen
            screenshot = pyautogui.screenshot(region=region)
            screenshot = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)
            
            # Convert both images to grayscale
            gray_screenshot = cv2.cvtColor(screenshot, cv2.COLOR_BGR2GRAY)
            gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            
            # Try multiple template matching methods
            methods = [
                cv2.TM_CCOEFF_NORMED,  # Best for most cases
                cv2.TM_CCORR_NORMED,   # Good for some cases where CCOEFF fails
            ]
            
            best_match = None
            best_confidence = 0
            
            for method in methods:
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
            
            if best_confidence >= self.confidence:
                # Calculate center coordinates
                h, w = gray_template.shape[:2]
                x = best_loc[0] + w // 2
                y = best_loc[1] + h // 2
                
                if region:
                    x += region[0]
                    y += region[1]
                    
                logger.debug(f"Found '{template_name}' at ({x}, {y}) with confidence {best_confidence:.4f}")
                return (x, y)
            else:
                logger.debug(f"Could not find '{template_name}' (best match confidence: {best_confidence:.4f})")
                # Save the screenshot and template for debugging
                debug_dir = os.path.join(os.path.dirname(__file__), 'debug')
                os.makedirs(debug_dir, exist_ok=True)
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(os.path.join(debug_dir, f'screenshot_{timestamp}.png'), screenshot)
                cv2.imwrite(os.path.join(debug_dir, f'template_{timestamp}.png'), template)
                logger.info(f"Saved debug images to {debug_dir}")
                return None
                
        except Exception as e:
            logger.error(f"Error finding template '{template_name}': {str(e)}", exc_info=True)
            return None
    
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
            confidence: Confidence threshold (0.0 to 1.0). If None, uses the bot's default confidence.
            
        Returns:
            Tuple of (x, y) coordinates of the center of the found image, or None if not found
        """
        if template_name not in self.templates:
            logger.error(f"Template '{template_name}' not found")
            return None
            
        # Use provided confidence or fall back to instance confidence
        conf_threshold = confidence if confidence is not None else self.confidence
        start_time = time.time()
        template = self.templates[template_name]
        
        while (time.time() - start_time) < timeout:
            try:
                # Take a screenshot of the specified region or full screen
                screenshot = pyautogui.screenshot(region=region) if region else pyautogui.screenshot()
                screenshot = cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)
                
                # Perform template matching
                result = cv2.matchTemplate(screenshot, template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                
                # Check if the match is above the confidence threshold
                if max_val >= conf_threshold:
                    # Calculate the center of the matched region
                    h, w = template.shape[:2]
                    x = max_loc[0] + w // 2
                    y = max_loc[1] + h // 2
                    
                    # Adjust coordinates if searching in a region
                    if region:
                        x += region[0]
                        y += region[1]
                        
                    logger.info(f"Found '{template_name}' at ({x}, {y}) with confidence {max_val:.2f}")
                    return (x, y)
                
                # Wait before next attempt
                time.sleep(check_interval)
                
            except Exception as e:
                logger.error(f"Error during image search: {e}")
                time.sleep(check_interval)
        
        logger.info(f"Template '{template_name}' not found after {timeout} seconds (confidence threshold: {conf_threshold:.2f})")
        return None
    
    def move_to(self, x: int, y: int, duration: float = 0.0) -> None:
        """
        Move mouse to the specified coordinates.
        
        Args:
            x: Target x-coordinate
            y: Target y-coordinate
            duration: Time in seconds for the movement. If 0, the movement is instant.
        """
        try:
            # Get current position for logging
            start_x, start_y = pyautogui.position()
            
            # Move the mouse
            if duration > 0:
                pyautogui.moveTo(x, y, duration=duration, tween=pyautogui.easeInOutQuad)
            else:
                pyautogui.moveTo(x, y, duration=0.1)  # Small duration to ensure smooth movement
            
            # Update current position
            self.current_position = (x, y)
            
            # Get final position for verification
            final_x, final_y = pyautogui.position()
            
            logger.info(f"Moved from ({start_x}, {start_y}) to ({x}, {y}) (final: {final_x}, {final_y}){f' over {duration:.2f}s' if duration > 0 else ''}")
            
            # If we didn't end up where we expected, log a warning
            if abs(final_x - x) > 5 or abs(final_y - y) > 5:  # Allow 5px tolerance
                logger.warning(f"Mouse did not reach target position. Expected: ({x}, {y}), Actual: ({final_x}, {final_y})")
                
        except Exception as e:
            logger.error(f"Error moving to ({x}, {y}): {str(e)}")
            # Try to update position even if there was an error
            try:
                final_pos = pyautogui.position()
                self.current_position = final_pos
                logger.info(f"Updated current position to {final_pos} after move error")
            except Exception as e2:
                logger.error(f"Failed to update position after error: {str(e2)}")
                
    def click_at(self, x: int, y: int, button: str = 'left', clicks: int = 1, force_move: bool = False) -> None:
        """
        Click at the specified coordinates.
        
        Args:
            x: X coordinate
            y: Y coordinate
            button: Mouse button ('left', 'middle', or 'right')
            clicks: Number of clicks
            force_move: If True, will move to the target position before clicking
        """
        try:
            current_x, current_y = pyautogui.position()
            
            # Only move if we're not already at the target position and force_move is True
            if (current_x, current_y) != (x, y):
                if force_move:
                    # For random region clicks, we want to move to the position
                    pyautogui.moveTo(x, y, duration=0.1)
                    time.sleep(0.1)  # Small delay to ensure movement is complete
                else:
                    # For regular clicks, just log the mismatch
                    logger.warning(f"Mouse not at target position. Current: ({current_x}, {current_y}), Target: ({x}, {y})")
                    logger.warning("Using current mouse position for click to prevent movement")
                    x, y = current_x, current_y
            
            # Log the click position
            logger.info(f"Clicking at position: ({x}, {y})")
            
            # Perform the click at the current position
            for _ in range(clicks):
                # Use direct mouse events for more control
                pyautogui.mouseDown(button=button)
                time.sleep(0.05)  # Short delay between down and up
                pyautogui.mouseUp(button=button)
                
                # Small delay between multiple clicks
                if _ < clicks - 1:
                    time.sleep(0.1)
            
            # Update current position
            self.current_position = (x, y)
            logger.info(f"Clicked at ({x}, {y}) with {button} button")
            
        except Exception as e:
            logger.error(f"Error clicking at ({x}, {y}): {str(e)}")
            raise
                
    def right_click_at(self, x: int, y: int) -> None:
        """Right click at the specified coordinates."""
        self.click_at(x, y, button='right')
        
    def double_click_at(self, x: int, y: int) -> None:
        """Double click at the specified coordinates."""
        self.click_at(x, y, clicks=2)
        
    def type_text(self, text: str) -> None:
        """Type the specified text."""
        try:
            pyautogui.write(text)
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
                self.click_at(x, y, button=action.button, clicks=action.clicks)
            elif action.type == ActionType.MOVE:
                self.move_to(x, y, duration=action.duration)
            elif action.type == ActionType.MOVE_TO:
                self.move_to(x, y, duration=action.duration)
            elif action.type == ActionType.RIGHT_CLICK:
                self.right_click_at(x, y)
            elif action.type == ActionType.DOUBLE_CLICK:
                self.double_click_at(x, y)
            elif action.type == ActionType.TYPE and action.text:
                self.type_text(action.text)
            elif action.type == ActionType.KEY_PRESS and action.key:
                self.press_key(action.key)
            elif action.type == ActionType.WAIT:
                self.wait(action.seconds)
            elif action.type == ActionType.SCROLL:
                self.scroll(action.pixels)
            elif action.type == ActionType.CLICK_AND_HOLD:
                try:
                    if self.current_position is None:
                        logger.error("No current position set for CLICK_AND_HOLD")
                        return False
                        
                    x, y = self.current_position
                    logger.info(f"Executing CLICK_AND_HOLD at current position: ({x}, {y}) for {action.duration} seconds")
                    
                    # Move to the position first
                    self.move_to(x, y, duration=0.1)
                    
                    # Perform the click and hold
                    pyautogui.mouseDown(button=action.button)
                    time.sleep(action.duration)
                    pyautogui.mouseUp(button=action.button)
                    
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
                self.current_position = (action.x, action.y)
                self.move_to(action.x, action.y, duration=action.duration)
                return True
            elif action.type == ActionType.MOVE_TO:
                try:
                    target_position = None
                    
                    # If we have a template to find
                    if hasattr(action, 'find') and action.find:
                        # Find the template position
                        position = self.find_template(action.find, action.confidence, action.timeout, action.region)
                        if position is None:
                            logger.error(f"Could not find template: {action.find}")
                            return False
                        target_position = position
                    # If we have explicit coordinates
                    elif hasattr(action, 'x') and action.x is not None and hasattr(action, 'y') and action.y is not None:
                        target_position = (action.x, action.y)
                    else:
                        logger.error("MOVE_TO action requires either a template to find or explicit coordinates")
                        return False
                    
                    # If random is enabled, pick a random point in the region
                    if getattr(action, 'random', False) and hasattr(action, 'random_region') and action.random_region:
                        region = action.random_region
                        x = random.randint(region[0], region[0] + region[2] - 1)
                        y = random.randint(region[1], region[1] + region[3] - 1)
                        target_position = (x, y)
                    
                    # Update and log the target position
                    self.current_position = target_position
                    logger.info(f"MOVE_TO target position set to: {target_position}")
                    
                    # Move to the position with the specified duration
                    self.move_to(target_position[0], target_position[1], duration=action.duration)
                    
                    # Verify the final position
                    final_pos = pyautogui.position()
                    logger.info(f"Final mouse position after move: {final_pos}")
                    
                    return True
                except Exception as e:
                    logger.error(f"Error in MOVE_TO action: {str(e)}")
                    return False
            elif action.type == ActionType.CLICK_AND_HOLD:
                # This should never be called directly - handled in execute_sequence
                logger.warning("CLICK_AND_HOLD action should be handled in execute_sequence")
                return False
            elif action.type == ActionType.CLICK:
                try:
                    button = getattr(action, 'button', 'left')
                    clicks = getattr(action, 'clicks', 1)
                    
                    # Check if this is a random region click
                    if getattr(action, 'random', False) and hasattr(action, 'random_region') and action.random_region:
                        # For random region clicks, calculate a random point in the region
                        region = action.random_region
                        x = random.randint(region[0], region[0] + region[2] - 1)
                        y = random.randint(region[1], region[1] + region[3] - 1)
                        logger.info(f"Executing RANDOM CLICK at: ({x}, {y}) in region {region}")
                        
                        # Move to the position and click
                        pyautogui.moveTo(x, y, duration=0.1)
                        time.sleep(0.1)  # Small delay to ensure movement is complete
                        
                        # Perform the click at the current position
                        for _ in range(clicks):
                            pyautogui.mouseDown(button=button)
                            time.sleep(0.05)
                            pyautogui.mouseUp(button=button)
                            if _ < clicks - 1:
                                time.sleep(0.1)
                                
                        # Update current position
                        self.current_position = (x, y)
                    else:
                        # For regular clicks, use current position without moving
                        x, y = pyautogui.position()
                        logger.info(f"Executing CLICK at current position: ({x}, {y})")
                        
                        # Use direct mouse events to avoid any movement
                        for _ in range(clicks):
                            pyautogui.mouseDown(button=button)
                            time.sleep(0.05)
                            pyautogui.mouseUp(button=button)
                            if _ < clicks - 1:
                                time.sleep(0.1)
                    
                    return True
                    
                except Exception as e:
                    logger.error(f"Error executing click: {str(e)}")
                    return False
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
