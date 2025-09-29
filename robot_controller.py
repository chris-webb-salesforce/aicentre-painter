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
        self.last_command_time = 0
        self.min_command_interval = 0.05  # Minimum 50ms between commands
        
        # Enable fresh mode for real-time commands (reduces command skipping)
        try:
            self.mc.set_fresh_mode(1)
            print("✅ Fresh mode enabled for real-time commands")
        except Exception as e:
            print(f"⚠️  Could not enable fresh mode: {e}")
        
        print("✅ Robot Controller initialized successfully")
    
    def interpolate_linear_path(self, start_coords, end_coords, max_segment_length=5.0):
        """
        Break a long move into smaller linear segments to reduce arcing.
        Returns list of intermediate coordinates.
        """
        if len(start_coords) < 3 or len(end_coords) < 3:
            return [end_coords]
        
        # Calculate distance
        distance = np.sqrt(sum((end_coords[i] - start_coords[i])**2 for i in range(3)))
        
        if distance <= max_segment_length:
            return [end_coords]  # Short enough, no interpolation needed
        
        # Calculate number of segments
        num_segments = int(np.ceil(distance / max_segment_length))
        
        # Generate intermediate points
        interpolated_points = []
        for i in range(1, num_segments + 1):
            t = i / num_segments
            interpolated_point = []
            for j in range(len(end_coords)):
                if j < 3:  # X, Y, Z coordinates
                    interpolated_point.append(start_coords[j] + t * (end_coords[j] - start_coords[j]))
                else:  # Orientation values, copy from end_coords
                    interpolated_point.append(end_coords[j])
            interpolated_points.append(interpolated_point)
        
        return interpolated_points
    
    def move_linear_interpolated(self, coords, speed, mode=0):
        """
        Move to coordinates using linear interpolation to minimize arcing.
        """
        try:
            # Get current position
            current_pos = self.mc.get_coords()
            if not current_pos or len(current_pos) < 3:
                print("⚠️  Could not get current position, using direct move")
                return self.synchronized_move(coords, speed, mode)
            
            # Generate interpolated path
            interpolated_points = self.interpolate_linear_path(current_pos, coords, max_segment_length=LINEAR_INTERPOLATION_MAX_SEGMENT)
            
            # Use faster movement method for interpolated points
            success = True
            for point in interpolated_points:
                if not self.synchronized_move(point, speed, mode, wait_for_completion=True):
                    print(f"⚠️  Failed to reach interpolated point: {point[:3]}")
                    success = False
                    break
            
            return success
            
        except Exception as e:
            print(f"❌ Linear interpolation failed: {e}")
            return self.synchronized_move(coords, speed, mode)

    def calculate_distance_compensation(self, x, y):
        """
        Calculate Z compensation based on distance from robot base.
        Further positions get higher Z values to compensate for arm deflection.
        """
        if not ENABLE_DISTANCE_COMPENSATION:
            return 0.0
        
        # Calculate distance from robot base
        distance = np.sqrt((x - BASE_POSITION_X)**2 + (y - BASE_POSITION_Y)**2)
        
        # Apply deadzone - no compensation for positions close to base
        if distance <= COMPENSATION_DEADZONE:
            return 0.0
        
        # Normalize distance beyond deadzone
        effective_distance = distance - COMPENSATION_DEADZONE
        max_effective_distance = MAX_REACH_DISTANCE - COMPENSATION_DEADZONE
        
        if max_effective_distance <= 0:
            return 0.0
        
        # Calculate normalized distance (0 to 1)
        normalized_distance = min(effective_distance / max_effective_distance, 1.0)
        
        # Apply compensation curve
        if Z_COMPENSATION_CURVE == "quadratic":
            compensation_factor = normalized_distance ** 2
        elif Z_COMPENSATION_CURVE == "cubic":
            compensation_factor = normalized_distance ** 3
        else:  # linear (default)
            compensation_factor = normalized_distance
        
        # Calculate final compensation
        compensation = compensation_factor * Z_COMPENSATION_MAX
        
        return compensation

    def apply_z_compensation(self, coords):
        """Apply distance-based Z compensation to coordinates."""
        if len(coords) < 3:
            return coords
        
        compensated_coords = coords.copy()
        x, y = coords[0], coords[1]
        
        # Calculate and apply compensation
        compensation = self.calculate_distance_compensation(x, y)
        compensated_coords[2] += compensation
        
        return compensated_coords

    def validate_z_coordinate(self, z):
        """Validate Z coordinate for safety and clamp to safe range."""
        if z < MIN_SAFE_Z:
            print(f"⚠️  WARNING: Z coordinate {z:.2f} below safety minimum {MIN_SAFE_Z:.2f}mm. Clamping to safe value.")
            return MIN_SAFE_Z + SAFETY_MARGIN
        elif z > MAX_SAFE_Z:
            print(f"⚠️  WARNING: Z coordinate {z:.2f} above safety maximum {MAX_SAFE_Z:.2f}mm. Clamping to safe value.")
            return MAX_SAFE_Z - SAFETY_MARGIN
        return z

    def safe_move_to_position(self, x, y, target_z=None, speed=TRAVEL_SPEED):
        """
        Safely move to a position by always traveling at safe height first.
        This prevents the robot from going below table level during position changes.
        """
        if target_z is None:
            target_z = SAFE_TRAVEL_HEIGHT
        
        # Validate target Z coordinate
        target_z = self.validate_z_coordinate(target_z)
        
        # Step 1: Always lift to safe travel height first (if not already there)
        current_pos = self.mc.get_coords()
        if current_pos and current_pos[2] < SAFE_TRAVEL_HEIGHT - 5:  # 5mm tolerance
            print(f"Lifting to safe travel height ({SAFE_TRAVEL_HEIGHT:.1f}mm)")
            safe_coords = [current_pos[0], current_pos[1], SAFE_TRAVEL_HEIGHT] + DRAWING_ORIENTATION
            compensated_coords = self.apply_z_compensation(safe_coords)
            self.mc.send_coords(compensated_coords, LIFT_SPEED, 0)
            
            if self.use_movement_sync:
                self.wait_for_movement_completion(compensated_coords[:3], timeout=2.0)
            else:
                time.sleep(1.0)
        
        # Step 2: Move to target X,Y at safe height
        travel_coords = [x, y, SAFE_TRAVEL_HEIGHT] + DRAWING_ORIENTATION
        compensated_coords = self.apply_z_compensation(travel_coords)
        self.mc.send_coords(compensated_coords, speed, 0)
        
        if self.use_movement_sync:
            self.wait_for_movement_completion(compensated_coords[:3])
        else:
            time.sleep(0.5)
        
        # Step 3: Move to final Z position if different from safe height
        if abs(target_z - SAFE_TRAVEL_HEIGHT) > 1.0:  # Only if significantly different
            final_coords = [x, y, target_z] + DRAWING_ORIENTATION
            compensated_coords = self.apply_z_compensation(final_coords)
            self.mc.send_coords(compensated_coords, APPROACH_SPEED, 0)
            
            if self.use_movement_sync:
                self.wait_for_movement_completion(compensated_coords[:3])
            else:
                time.sleep(0.3)
        
        return True

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
    
    def wait_until_stopped(self, timeout=None):
        """Wait until robot has completely stopped moving."""
        if timeout is None:
            timeout = self.max_wait_time
        
        start_time = time.time()
        stable_readings = 0
        required_stable_readings = 5  # Must be stable for 5 consecutive readings
        last_position = None
        
        while time.time() - start_time < timeout:
            try:
                # Check if robot is in motion using built-in method
                if hasattr(self.mc, 'is_in_position') and callable(self.mc.is_in_position):
                    try:
                        # Try different parameter formats for is_in_position
                        current_pos = self.mc.get_coords()
                        if current_pos:
                            in_position = self.mc.is_in_position(current_pos, error=1.0)
                            if in_position:
                                stable_readings += 1
                                if stable_readings >= required_stable_readings:
                                    return True
                            else:
                                stable_readings = 0
                        else:
                            stable_readings = 0
                    except Exception:
                        # Fallback to manual position checking
                        pass
                
                # Manual position stability check
                current_pos = self.mc.get_coords()
                if current_pos and len(current_pos) >= 3:
                    if last_position:
                        # Calculate movement between readings
                        movement = np.sqrt(sum((current_pos[i] - last_position[i])**2 for i in range(3)))
                        if movement < 0.1:  # Very small movement threshold
                            stable_readings += 1
                            if stable_readings >= required_stable_readings:
                                time.sleep(0.02)  # Final settling delay
                                return True
                        else:
                            stable_readings = 0
                    last_position = current_pos[:]
                
                time.sleep(0.02)  # Fast polling
                
            except Exception as e:
                print(f"⚠️  Motion detection error: {e}")
                time.sleep(0.05)
                break
        
        print(f"⚠️  Stop detection timeout after {timeout}s")
        return False

    def wait_for_movement_completion(self, target_position, timeout=None):
        """Wait for robot to reach target position before proceeding."""
        if not self.use_movement_sync:
            time.sleep(0.05)  # Minimum delay even without sync
            return True
        
        if timeout is None:
            timeout = self.max_wait_time
        
        # First, wait until robot stops moving
        stop_success = self.wait_until_stopped(timeout * 0.8)  # Use 80% of timeout
        
        if not stop_success:
            print("⚠️  Robot may still be moving")
            return False
        
        # Then verify we're at the target position
        try:
            current_pos = self.mc.get_coords()
            if current_pos and len(current_pos) >= 3 and len(target_position) >= 3:
                distance = np.sqrt(sum((current_pos[i] - target_position[i])**2 for i in range(3)))
                if distance <= self.position_tolerance:
                    return True
                else:
                    print(f"⚠️  Position error: {distance:.2f}mm from target")
                    return False
        except Exception as e:
            print(f"⚠️  Position check failed: {e}")
            
        return True  # Assume success if we can't verify
    
    def move_and_wait_stopped(self, coords, speed, mode=0, timeout=None):
        """Move to position and wait until completely stopped."""
        try:
            # Enforce minimum command interval
            current_time = time.time()
            time_since_last = current_time - self.last_command_time
            if time_since_last < self.min_command_interval:
                time.sleep(self.min_command_interval - time_since_last)
            
            # Apply safety validation and compensation
            safe_coords = coords.copy()
            if len(safe_coords) >= 3:
                safe_coords[2] = self.validate_z_coordinate(safe_coords[2])
            compensated_coords = self.apply_z_compensation(safe_coords)
            
            # Send movement command
            self.mc.send_coords(compensated_coords, speed, mode)
            self.last_command_time = time.time()
            
            # Wait until completely stopped
            if timeout is None:
                timeout = self.max_wait_time
            
            return self.wait_until_stopped(timeout)
            
        except Exception as e:
            print(f"❌ Move and wait failed: {e}")
            return False
    
    def synchronized_move(self, coords, speed, mode=0, wait_for_completion=True):
        """Send movement command and optionally wait for completion."""
        try:
            # Enforce minimum command interval to prevent overrun
            current_time = time.time()
            time_since_last = current_time - self.last_command_time
            if time_since_last < self.min_command_interval:
                time.sleep(self.min_command_interval - time_since_last)
            
            # Apply safety validation to Z coordinate
            safe_coords = coords.copy()
            if len(safe_coords) >= 3:
                safe_coords[2] = self.validate_z_coordinate(safe_coords[2])
            
            # Apply distance-based Z compensation
            compensated_coords = self.apply_z_compensation(safe_coords)
            self.mc.send_coords(compensated_coords, speed, mode)
            self.last_command_time = time.time()
            
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
        
        # Calculate base Z positions (compensation will be applied in synchronized_move)
        base_retract_z = PEN_RETRACT_Z
        base_drawing_z = PEN_DRAWING_Z
        
        # SAFETY: Use safe movement to get to position
        print(f"Moving safely to position ({x:.1f}, {y:.1f})")
        target_coords = [x, y, base_retract_z] + DRAWING_ORIENTATION
        # Use appropriate movement mode for travel
        self.synchronized_move(target_coords, TRAVEL_SPEED, TRAVEL_MOVEMENT_MODE)
        
        # Gradual approach to drawing surface in steps
        approach_steps = 3
        for i in range(1, approach_steps + 1):
            z_position = base_retract_z - ((base_retract_z - base_drawing_z) * (i / approach_steps))
            self.synchronized_move([x, y, z_position] + DRAWING_ORIENTATION, int(APPROACH_SPEED // 2), 0)
        
        # Final positioning at drawing depth
        self.synchronized_move([x, y, base_drawing_z] + DRAWING_ORIENTATION, int(APPROACH_SPEED // 3), 0)
        
        self.pen_is_down = True
        self.current_position = [x, y]
        # Store the compensated Z depth for later reference
        compensation = self.calculate_distance_compensation(x, y)
        self.current_z_depth = base_drawing_z + compensation
        self.movement_counter = 0
    
    def gentle_pen_up(self):
        """Gently lift the pen from the drawing surface with smooth transition."""
        if not self.pen_is_down:
            return
            
        current = self.mc.get_coords()
        if current:
            # SAFETY: Always lift to safe retract height
            print(f"Lifting pen safely from ({current[0]:.1f}, {current[1]:.1f})")
            
            # Gradual lift in steps to prevent jerky movement
            lift_steps = 2
            for i in range(1, lift_steps + 1):
                z_position = self.current_z_depth + ((PEN_RETRACT_Z - self.current_z_depth) * (i / lift_steps))
                # Validate each lift step for safety
                z_position = self.validate_z_coordinate(z_position)
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
        
        # Use appropriate movement mode for drawing
        target_coords = [x2, y2, PEN_DRAWING_Z] + DRAWING_ORIENTATION
        
        if DRAWING_MOVEMENT_MODE == 1:
            # Use robot's built-in linear interpolation (mode 1) - much more efficient
            self.synchronized_move(target_coords, DRAWING_SPEED, DRAWING_MOVEMENT_MODE)
        else:
            # Use standard joint interpolation (mode 0) for speed
            self.synchronized_move(target_coords, DRAWING_SPEED, 0)
        
        self.current_position = [x2, y2]

    def go_to_home_position(self):
        """Move to drawing start position."""
        print("Moving to drawing start position...")
        self.gentle_pen_up()
        # SAFETY: Use safe movement to home position
        self.safe_move_to_position(ORIGIN_X, ORIGIN_Y, PEN_RETRACT_Z, 40)
    
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

    def test_distance_compensation(self):
        """Test and visualize the distance compensation at various positions."""
        print("\n--- DISTANCE COMPENSATION TEST ---")
        print(f"Compensation enabled: {ENABLE_DISTANCE_COMPENSATION}")
        print(f"Max compensation: {Z_COMPENSATION_MAX}mm")
        print(f"Compensation curve: {Z_COMPENSATION_CURVE}")
        print(f"Deadzone: {COMPENSATION_DEADZONE}mm")
        print()
        
        # Test positions at various distances
        test_positions = [
            (ORIGIN_X, ORIGIN_Y),  # At origin
            (ORIGIN_X + 50, ORIGIN_Y + 50),   # Close to base
            (ORIGIN_X + 150, ORIGIN_Y + 100), # Medium distance
            (ORIGIN_X + 250, ORIGIN_Y + 150), # Far from base
            (ORIGIN_X + 350, ORIGIN_Y + 200), # Very far
        ]
        
        print("Position Test Results:")
        print("X(mm)    Y(mm)    Distance(mm)  Compensation(mm)")
        print("-" * 50)
        
        for x, y in test_positions:
            distance = np.sqrt((x - BASE_POSITION_X)**2 + (y - BASE_POSITION_Y)**2)
            compensation = self.calculate_distance_compensation(x, y)
            print(f"{x:6.1f}   {y:6.1f}   {distance:8.1f}     {compensation:8.3f}")
        
        print()
        
        # Show compensation curve visualization
        print("Distance vs Compensation curve:")
        distances = [0, 50, 100, 150, 200, 250, 300, 350, 400]
        for dist in distances:
            # Test position at this distance
            x = BASE_POSITION_X + dist
            y = BASE_POSITION_Y
            comp = self.calculate_distance_compensation(x, y)
            bar_length = int(comp * 10)  # Scale for visualization
            bar = "█" * bar_length
            print(f"{dist:3.0f}mm: {comp:5.2f}mm |{bar}")

    def calibrate_compensation_interactively(self):
        """Interactive calibration of Z compensation parameters."""
        print("\n--- INTERACTIVE COMPENSATION CALIBRATION ---")
        print("This will help you find optimal compensation values.")
        print()
        
        # Move to a test position far from base
        test_x = ORIGIN_X + 200  # 200mm from origin
        test_y = ORIGIN_Y + 150  # 150mm from origin
        test_distance = np.sqrt((test_x - BASE_POSITION_X)**2 + (test_y - BASE_POSITION_Y)**2)
        
        print(f"Moving to test position: ({test_x:.1f}, {test_y:.1f})")
        print(f"Distance from base: {test_distance:.1f}mm")
        print()
        
        # Move to position with current compensation
        self.go_to_home_position()
        time.sleep(1)
        
        print("Testing current compensation...")
        self.gentle_pen_down(test_x, test_y)
        time.sleep(2)
        self.gentle_pen_up()
        
        print("\nAdjust compensation parameters in config.py:")
        print(f"Current Z_COMPENSATION_MAX: {Z_COMPENSATION_MAX}")
        print(f"Current Z_COMPENSATION_CURVE: {Z_COMPENSATION_CURVE}")
        print(f"Current COMPENSATION_DEADZONE: {COMPENSATION_DEADZONE}")
        
        print("\nRecommendations:")
        print("- If pen is too low (digging): Increase Z_COMPENSATION_MAX")
        print("- If pen is too high (not touching): Decrease Z_COMPENSATION_MAX") 
        print("- For gradual compensation: Use 'linear' curve")
        print("- For rapid compensation at distance: Use 'quadratic' or 'cubic'")
        
        self.go_to_home_position()

    def test_safety_system(self):
        """Test the safety system to ensure it prevents dangerous movements."""
        print("\n--- SAFETY SYSTEM TEST ---")
        print(f"Safe travel height: {SAFE_TRAVEL_HEIGHT}mm")
        print(f"Minimum safe Z: {MIN_SAFE_Z}mm") 
        print(f"Maximum safe Z: {MAX_SAFE_Z}mm")
        print()
        
        # Test 1: Safe position movement
        print("Test 1: Safe position movement")
        test_positions = [
            (ORIGIN_X + 100, ORIGIN_Y + 50),
            (ORIGIN_X - 50, ORIGIN_Y + 100),
            (ORIGIN_X + 150, ORIGIN_Y - 30),
        ]
        
        for i, (x, y) in enumerate(test_positions):
            print(f"  Moving safely to position {i+1}: ({x:.1f}, {y:.1f})")
            try:
                self.safe_move_to_position(x, y)
                print(f"  ✅ Position {i+1} completed safely")
                time.sleep(0.5)
            except Exception as e:
                print(f"  ❌ Position {i+1} failed: {e}")
        
        print()
        
        # Test 2: Z coordinate validation
        print("Test 2: Z coordinate validation")
        test_z_values = [
            ORIGIN_Z - 20,  # Too low (should be clamped)
            ORIGIN_Z + 300, # Too high (should be clamped)
            ORIGIN_Z,       # Normal
            PEN_DRAWING_Z,  # Normal drawing height
        ]
        
        for z in test_z_values:
            validated_z = self.validate_z_coordinate(z)
            status = "✅ OK" if validated_z == z else f"⚠️  Clamped from {z:.1f} to {validated_z:.1f}"
            print(f"  Z={z:6.1f}mm → {validated_z:6.1f}mm {status}")
        
        print()
        
        # Test 3: Safe pen operations
        print("Test 3: Safe pen operations")
        test_x, test_y = ORIGIN_X + 80, ORIGIN_Y + 60
        
        try:
            print(f"  Testing safe pen down at ({test_x:.1f}, {test_y:.1f})")
            self.gentle_pen_down(test_x, test_y)
            time.sleep(1)
            
            print("  Testing safe pen up")
            self.gentle_pen_up()
            time.sleep(1)
            
            print("  ✅ Safe pen operations completed")
        except Exception as e:
            print(f"  ❌ Safe pen operations failed: {e}")
        
        # Return to safe position
        self.go_to_home_position()
        print("✅ Safety system test completed")