#!/usr/bin/env python3
"""
Test script to demonstrate the progress bar functionality
without actually running the robot
"""

import time
import sys

def print_progress_bar(current, total, bar_length=50, prefix="Progress"):
    """Print a progress bar to the terminal."""
    if total == 0:
        return
    
    progress = current / total
    filled_length = int(bar_length * progress)
    bar = '█' * filled_length + '░' * (bar_length - filled_length)
    percent = progress * 100
    
    # Print with carriage return to update same line
    print(f'\r{prefix}: |{bar}| {percent:.1f}% ({current}/{total})', end='', flush=True)
    
    # Print newline when complete
    if current >= total:
        print()

def simulate_drawing():
    """Simulate the drawing process with progress bar."""
    print("\n--- MyCobot Vertical Drawing System ---")
    print("Simulating drawing process...\n")
    
    total_contours = 150  # Simulate 150 contours
    print(f"Found {total_contours} contours to draw.")
    print("="*60)
    
    start_time = time.time()
    
    for i in range(1, total_contours + 1):
        # Update progress bar
        print_progress_bar(i, total_contours, prefix="Drawing")
        
        # Simulate drawing time
        time.sleep(0.05)  # 50ms per contour for demo
        
        # Simulate rest periods
        if i % 30 == 0 and i < total_contours:
            print()  # New line
            print(f"\n💤 Resting for 3 seconds...")
            time.sleep(1)  # Shorter rest for demo
            print("Resuming drawing...")
            print()
    
    # Final summary
    print()  # New line after progress bar
    elapsed_time = time.time() - start_time
    print("="*60)
    print(f"\n✓ Drawing complete!")
    print(f"  Total contours drawn: {total_contours}")
    print(f"  Time elapsed: {elapsed_time//60:.0f}m {elapsed_time%60:.0f}s")
    print("="*60)

def demo_different_speeds():
    """Show progress bar at different speeds."""
    print("\n--- Progress Bar Speed Demo ---\n")
    
    speeds = [
        ("Fast", 0.01, 50),
        ("Medium", 0.05, 30),
        ("Slow", 0.1, 20)
    ]
    
    for name, delay, count in speeds:
        print(f"\n{name} speed ({count} items):")
        for i in range(1, count + 1):
            print_progress_bar(i, count, prefix=f"{name}")
            time.sleep(delay)
        time.sleep(0.5)

if __name__ == "__main__":
    print("Progress Bar Demonstration")
    print("="*40)
    
    # Run main simulation
    simulate_drawing()
    
    # Show different speeds
    demo_different_speeds()
    
    print("\n✅ Demo complete!")