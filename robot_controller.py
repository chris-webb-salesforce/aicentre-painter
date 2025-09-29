"""
Core robot control module for MyCobot 320.
Handles all robot movement, positioning, and state management.
"""

import time
import numpy as np
from pymycobot import MyCobot320
from config import *


class RobotController:
    """Core robot control functionality."""
    
    def __init__(self):
        """Initialize robot controller."""
        print("Initializing Robot Controller...")
        try:
            self.mc = MyCobot320(SERIAL_PORT, BAUD_RATE)
            time.sleep(2)  # Give robot time to initialize
            
            # Enable adaptive mode for smoother movements
            self.mc.set_fresh_mode(1)  # Enable coordinate refresh mode
            
        except Exception as e:
            print(f"❌ Failed to connect to robot: {e}")
            raise
        
        # Track current pen state and depth
        self.pen_is_down = False
        self.current_position = None
        self.current_z_depth = PEN_RETRACT_Z
        self.movement_counter = 0
        
        # Force feedback monitoring
        self.last_force_check = time.time()
        self.force_warnings = 0
        
        # Movement synchronization settings
        self.use_movement_sync = USE_MOVEMENT_SYNC
        self.max_wait_time = MAX_WAIT_TIME
        self.position_tolerance = POSITION_TOLERANCE
        self.position_check_interval = POSITION_CHECK_INTERVAL
        
        print("✅ Robot Controller initialized successfully")

    def check_force_feedback(self):
        """Monitor force feedback and adjust Z if needed."""
        current_time = time.time()
        if current_time - self.last_force_check < FORCE_CHECK_INTERVAL:
            return
        
        self.last_force_check = current_time
        
        try:
            # In a real implementation, you would check robot force sensors here
            # For now, we'll use position-based depth compensation
            if self.pen_is_down and self.movement_counter > 75:
                # After many movements, check for excessive drift
                z_drift = PEN_DRAWING_Z - self.current_z_depth
                if abs(z_drift) > Z_STABILIZATION_THRESHOLD:
                    # Apply gentle correction
                    correction = z_drift * 0.1  # 10% correction
                    self.current_z_depth += correction
                    # Clamp to safe range
                    self.current_z_depth = max(PEN_DRAWING_Z - 0.8, 
                                              min(PEN_DRAWING_Z + 0.4, self.current_z_depth))
                    print(f"Force-based Z correction: {self.current_z_depth:.2f}mm (drift: {z_drift:.2f}mm)")
        except Exception:
            # Force feedback not available - continue with position control
            pass
    
    def stabilize_arm_position(self):
        """Add a brief pause to let arm stabilize and reduce oscillation."""
        # Get current position to check if arm has settled
        current = self.mc.get_coords()
        if current:
            # Brief stabilization pause
            time.sleep(0.02)
    
    def wait_for_movement_completion(self, target_position, timeout=None):
        """Wait for robot to reach target position before proceeding."""
        if not self.use_movement_sync:
            return True
        
        if timeout is None:
            timeout = self.max_wait_time
        
        start_time = time.time()
        consecutive_failures = 0
        
        while time.time() - start_time < timeout:
            try:
                current_pos = self.mc.get_coords()
                if current_pos and len(current_pos) >= 3 and len(target_position) >= 3:
                    # Calculate distance to target (only check X, Y, Z)
                    distance = np.sqrt(sum((current_pos[i] - target_position[i])**2 for i in range(3)))
                    
                    if distance <= self.position_tolerance:
                        return True
                    
                    consecutive_failures = 0  # Reset failure counter
                    time.sleep(self.position_check_interval)  # Fast checking
                else:
                    consecutive_failures += 1
                    if consecutive_failures > 5:  # If get_coords fails repeatedly, give up early
                        print("⚠️  Position checking failed - using fallback timing")
                        time.sleep(0.1)  # Short fallback
                        return True
                    time.sleep(0.02)
                    
            except Exception:
                consecutive_failures += 1
                if consecutive_failures > 5:
                    print("⚠️  Position checking error - using fallback timing") 
                    time.sleep(0.1)  # Short fallback
                    return True
                time.sleep(0.02)
                
        print(f"⚠️  Movement timeout after {timeout}s - continuing anyway")
        return False
    
    def synchronized_move(self, coords, speed, mode=0, wait_for_completion=True):
        """Send movement command and optionally wait for completion."""
        try:
            self.mc.send_coords(coords, speed, mode)
            
            if wait_for_completion and self.use_movement_sync:
                # For drawing movements, use shorter timeout 
                if speed >= DRAWING_SPEED:  # Drawing or faster movements
                    timeout = min(1.5, self.max_wait_time)  # Max 1.5 seconds for drawing
                else:
                    timeout = self.max_wait_time  # Full timeout for positioning moves
                
                # Wait for movement to complete
                success = self.wait_for_movement_completion(coords[:3], timeout)
                return success
            else:
                # Use smart fallback timing if sync is disabled
                movement_time = max(0.05, min(0.5, speed / 200.0))  # Faster estimates
                time.sleep(movement_time)
                return True
                
        except Exception as e:
            print(f"❌ Movement command failed: {e}")
            return False
    
    def synchronized_angles(self, angles, speed, wait_for_completion=True):
        """Send angle command and optionally wait for completion.""" 
        try:
            self.mc.send_angles(angles, speed)
            
            if wait_for_completion and self.use_movement_sync:
                # For angle movements, use time-based waiting since position checking is harder
                movement_time = max(1.0, min(3.0, speed / 30.0))  # Faster, capped estimates
                time.sleep(movement_time)
                return True
            else:
                time.sleep(max(0.5, min(2.0, speed / 40.0)))  # Faster fallback
                return True
                
        except Exception as e:
            print(f"❌ Angle command failed: {e}")
            return False

    def gentle_pen_down(self, x, y):
        """Gently lower the pen to the drawing surface with smooth transition."""
        if self.pen_is_down:
            return
            
        # Move to position above the point
        self.synchronized_move([x, y, PEN_RETRACT_Z] + DRAWING_ORIENTATION, TRAVEL_SPEED, 0)
        
        # Gradual approach to drawing surface in steps
        approach_steps = 3
        for i in range(1, approach_steps + 1):
            z_position = PEN_RETRACT_Z - ((PEN_RETRACT_Z - PEN_DRAWING_Z) * (i / approach_steps))
            self.synchronized_move([x, y, z_position] + DRAWING_ORIENTATION, int(APPROACH_SPEED // 2), 0)
        
        # Final positioning at drawing depth
        self.synchronized_move([x, y, PEN_DRAWING_Z] + DRAWING_ORIENTATION, int(APPROACH_SPEED // 3), 0)
        
        self.pen_is_down = True
        self.current_position = [x, y]
        self.current_z_depth = PEN_DRAWING_Z
        self.movement_counter = 0
    
    def gentle_pen_up(self):
        """Gently lift the pen from the drawing surface with smooth transition."""
        if not self.pen_is_down:
            return
            
        current = self.mc.get_coords()
        if current:
            # Gradual lift in steps to prevent jerky movement
            lift_steps = 2
            for i in range(1, lift_steps + 1):
                z_position = self.current_z_depth + ((PEN_RETRACT_Z - self.current_z_depth) * (i / lift_steps))
                self.synchronized_move([current[0], current[1], z_position] + DRAWING_ORIENTATION, int(LIFT_SPEED // 2), 0)
        
        self.pen_is_down = False
        self.current_z_depth = PEN_RETRACT_Z

    def draw_line_segment(self, from_point, to_point):
        """Draw a line segment with smooth interpolation and depth control."""
        x1, y1 = from_point
        x2, y2 = to_point
        
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Enhanced Z compensation to prevent digging deeper
        self.movement_counter += 1
        if self.movement_counter % 15 == 0:  # Every 15 movements (less frequent)
            # Check if we've drifted too far down
            z_drift = PEN_DRAWING_Z - self.current_z_depth
            if z_drift > MAX_Z_DRIFT:
                # Gradual correction back towards target depth
                self.current_z_depth = (self.current_z_depth * Z_COMPENSATION_FACTOR + 
                                       PEN_DRAWING_Z * (1 - Z_COMPENSATION_FACTOR))
                # Clamp to safe range
                self.current_z_depth = max(PEN_DRAWING_Z - 0.5, 
                                          min(PEN_DRAWING_Z + 0.3, self.current_z_depth))
        
        if distance < MIN_SEGMENT_LENGTH:
            # For short segments, single smooth movement
            self.synchronized_move([x2, y2, self.current_z_depth] + DRAWING_ORIENTATION, DRAWING_SPEED, 0)
        else:
            # More interpolation points for smoother curves
            num_points = max(2, min(INTERPOLATION_POINTS, int(distance / MIN_SEGMENT_LENGTH)))
            for i in range(1, num_points + 1):
                ratio = i / num_points
                x = x1 + (x2 - x1) * ratio
                y = y1 + (y2 - y1) * ratio
                
                # Smooth speed ramping for better control
                if i == 1 or i == num_points:
                    speed = int(DRAWING_SPEED * 0.8)  # Slower at segment endpoints
                else:
                    speed = DRAWING_SPEED
                
                # Synchronized movement with proper waiting
                self.synchronized_move([x, y, self.current_z_depth] + DRAWING_ORIENTATION, speed, 0)
                
                # Check force feedback and stabilize periodically
                if i % 2 == 0:  # Every other point
                    self.check_force_feedback()
                    self.stabilize_arm_position()
        
        self.current_position = [x2, y2]

    def go_to_home_position(self):
        """Move to drawing start position."""
        print("Moving to drawing start position...")
        self.gentle_pen_up()
        # Position robot ready for drawing on flat table
        initial_coords = [ORIGIN_X, ORIGIN_Y, PEN_RETRACT_Z] + DRAWING_ORIENTATION
        self.synchronized_move(initial_coords, 40, 0)
    
    def go_to_photo_position(self):
        """Move to photo capture position."""
        print("Moving to photo position...")
        self.synchronized_angles([0, -45, -45, 0, 90, 0], 40)
    
    def go_to_safe_position(self):
        """Move to neutral safe position when not drawing."""
        print("Moving to safe neutral position...")
        self.gentle_pen_up()
        self.synchronized_angles([0, 0, 0, 0, 90, DESIRED_J6_ANGLE], 40)  # J6 at desired angle

    def get_current_position(self):
        """Get current robot position."""
        try:
            angles = self.mc.get_angles()
            coords = self.mc.get_coords()
            return {"angles": angles, "coords": coords}
        except Exception as e:
            print(f"❌ Failed to get current position: {e}")
            return None