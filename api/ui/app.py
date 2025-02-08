import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))  # Add project root to Python path

import tkinter as tk
from tkinter import messagebox
import sys
import os

# Add parent directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from api.core.swarm import Swarm
from api.core.hopfield import HopfieldNetwork
import numpy as np

class SwarmVisualizer:
    def __init__(self, master, swarm, rows=5, cols=3):
        self.master = master
        self.swarm = swarm
        self.canvas_size = 600
        
        # Store grid dimensions
        self.rows = rows
        self.cols = cols
        
        # Setup main canvas
        self.canvas = tk.Canvas(master, width=self.canvas_size, 
                              height=self.canvas_size, bg='white')
        self.canvas.pack(padx=10, pady=10)
        
        # Control buttons and settings
        self.controls = tk.Frame(master)
        self.controls.pack(pady=5)
        
        # Settings frames
        self.settings_frame = tk.Frame(self.controls)
        self.settings_frame.pack(side=tk.TOP, pady=5)
        
        # Left settings frame
        self.left_settings = tk.Frame(self.settings_frame)
        self.left_settings.pack(side=tk.LEFT, padx=10)
        
        # Swarm size controls
        tk.Label(self.left_settings, text="Rows:").pack(side=tk.LEFT)
        self.rows_entry = tk.Spinbox(self.left_settings, from_=1, to=10, width=3)
        self.rows_entry.pack(side=tk.LEFT, padx=5)
        self.rows_entry.delete(0, "end")
        self.rows_entry.insert(0, str(rows))
        
        tk.Label(self.left_settings, text="Cols:").pack(side=tk.LEFT)
        self.cols_entry = tk.Spinbox(self.left_settings, from_=1, to=10, width=3)
        self.cols_entry.pack(side=tk.LEFT, padx=5)
        self.cols_entry.delete(0, "end")
        self.cols_entry.insert(0, str(cols))
        
        # Turn direction selection
        tk.Label(self.left_settings, text="Turn Direction:").pack(side=tk.LEFT)
        self.pattern_var = tk.StringVar(value="Left")
        self.left_turn = tk.Radiobutton(self.left_settings, text="Left", variable=self.pattern_var, 
                                      value="Left", command=self.set_pattern)
        self.right_turn = tk.Radiobutton(self.left_settings, text="Right", variable=self.pattern_var, 
                                       value="Right", command=self.set_pattern)
        self.left_turn.pack(side=tk.LEFT, padx=5)
        self.right_turn.pack(side=tk.LEFT, padx=5)
        
        # Controller selection dropdown
        self.controller_label = tk.Label(self.controls, text="Controller Type:")
        self.controller_label.pack(side=tk.TOP)
        self.controller_type = tk.StringVar(value="Rigid Body")
        self.controller_options = ["Rigid Body", "Hopfield Network"]
        self.controller_dropdown = tk.OptionMenu(self.controls, self.controller_type, *self.controller_options)
        self.controller_dropdown.pack(side=tk.TOP, pady=5)
        
        # Control buttons frame
        self.buttons_frame = tk.Frame(self.controls)
        self.buttons_frame.pack(side=tk.TOP, pady=5)
        
        # Control buttons
        self.start_sim_btn = tk.Button(self.buttons_frame, text="Start Simulation", command=self.start)
        self.stop_sim_btn = tk.Button(self.buttons_frame, text="Stop Simulation", command=self.stop)
        self.reset_btn = tk.Button(self.buttons_frame, text="Reset", command=self.reset)
        self.train_btn = tk.Button(self.buttons_frame, text="Train Network", command=self.auto_train_patterns)
        
        self.start_sim_btn.pack(side=tk.LEFT, padx=2)
        self.stop_sim_btn.pack(side=tk.LEFT, padx=2)
        self.reset_btn.pack(side=tk.LEFT, padx=2)
        self.train_btn.pack(side=tk.LEFT, padx=2)
        
        # Initialize center trail
        self.center_trail = []
        
        # Drawing parameters
        self.robot_radius = 5
        self.robot_color = 'blue'
        self.running = False
        
        # Initialize patterns
        self.swarm._initialize_patterns()
        
        # Initial draw
        self.draw_robots()

    def draw_robots(self):
        """Draw robots and formation center with trail"""
        self.canvas.delete("all")
        
        # Draw trail
        for i in range(1, len(self.center_trail)):
            x1, y1 = self.scale_position(self.center_trail[i-1])
            x2, y2 = self.scale_position(self.center_trail[i])
            self.canvas.create_line(x1, y1, x2, y2, fill='red', dash=(4, 2))
        
        # Draw robots
        positions = []
        for robot in self.swarm.robots:
            x, y = self.scale_position(robot.position)
            positions.append(robot.position)
            self.canvas.create_oval(
                x - self.robot_radius, y - self.robot_radius,
                x + self.robot_radius, y + self.robot_radius,
                fill=self.robot_color
            )
        
        # Calculate and draw formation center
        center = np.mean(positions, axis=0)
        x, y = self.scale_position(center)
        self.canvas.create_oval(
            x - 3, y - 3,
            x + 3, y + 3,
            fill='red'
        )
        
        # Update center trail
        self.center_trail.append(center)
        # Keep only last 50 positions
        if len(self.center_trail) > 50:
            self.center_trail.pop(0)

    def scale_position(self, pos):
        """Scale swarm coordinates to canvas pixels"""
        scale = self.canvas_size / 20  # Assuming swarm operates in [-10,10] space
        return (
            pos[0] * scale + self.canvas_size/2,
            self.canvas_size/2 - pos[1] * scale
        )

    def set_pattern(self):
        """Set the movement pattern (left/right turn)"""
        try:
            pattern = 0 if self.pattern_var.get() == "Left" else 1
            print(f"[DEBUG] Setting pattern to: {'Left' if pattern == 0 else 'Right'}")
            self.swarm.set_pattern(pattern)
            # Update UI to match actual pattern index after validation
            self.pattern_var.set("Left" if self.swarm.current_pattern == 0 else "Right")
            # Always start running when pattern changes
            self.running = True
            self.update()
        except Exception as e:
            messagebox.showerror("Pattern Error", 
                f"Failed to set pattern: {str(e)}\nPlease train patterns first.")
            self.running = False

    def start(self):
        self.running = True
        self.update()

    def stop(self):
        self.running = False

    def reset(self):
        """Reset swarm"""
        self.swarm = Swarm(rows=self.rows, cols=self.cols)
        self.swarm._initialize_patterns()  # Ensure patterns are initialized
        self.center_trail = []
        self.draw_robots()

    def update(self):
        if self.running:
            print("[DEBUG] UI Update tick")
            self.swarm.update()
            self.draw_robots()
            self.master.after(50, self.update)

    def auto_train_patterns(self):
        """Train patterns for left and right turns"""
        # Force pattern reinitialization
        self.swarm._initialize_patterns()
        messagebox.showinfo("Success", "Patterns initialized for left and right turns")

if __name__ == '__main__':
    root = tk.Tk()
    root.title("Swarm Visualization")
    
    # Configure grid size
    rows, cols = 2, 2
    swarm = Swarm(rows=rows, cols=cols)
    visualizer = SwarmVisualizer(root, swarm, rows=rows, cols=cols)
    
    root.mainloop()
