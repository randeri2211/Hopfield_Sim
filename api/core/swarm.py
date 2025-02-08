import numpy as np
from .differential_robot import DifferentialRobot
from .hopfield_control import SwarmHopfieldControl

class Swarm:
    """Manages a swarm of robots with integrated Hopfield pattern control"""
    def __init__(self, rows=5, cols=3, speed=0.2, angular_speed=0.1):
        """
        Initialize swarm with Hopfield network integration.

        Args:
            rows: Number of rows in the grid.
            cols: Number of columns in the grid.
            speed: Fixed speed of the swarm's center.
            angular_speed: Angular speed for turning.
        """
        from .hopfield import create_grid_positions

        # Store initialization parameters
        self.rows = rows
        self.cols = cols
        self.speed = speed
        self.angular_speed = angular_speed

        # Create robot instances and Hopfield control
        self._create_robots(rows, cols)
        self._initialize_patterns()

    def _create_robots(self, rows, cols):
        """Initialize robot positions in grid pattern"""
        from .hopfield import create_grid_positions
        grid_positions = create_grid_positions(rows, cols)
        self.robots = [DifferentialRobot(i, (x, y)) for i, (x, y) in enumerate(grid_positions)]

        # Initialize Hopfield control system
        self.hopfield = SwarmHopfieldControl(
            robot_positions=grid_positions,
            speed=self.speed,
            angular_speed=self.angular_speed
        )

        self.current_pattern = 0  # 0 for left turn, 1 for right turn

    def set_pattern(self, pattern_idx: int):
        """Set movement pattern (0 for left turn, 1 for right turn)"""
        if not hasattr(self.hopfield, 'velocity_patterns'):
            self._initialize_patterns()
            
        if 0 <= pattern_idx < len(self.hopfield.velocity_patterns):
            print(f"[DEBUG] Setting pattern to {pattern_idx}")
            self.current_pattern = pattern_idx
        else:
            print(f"[ERROR] Invalid pattern index {pattern_idx}, resetting to 0")
            self.current_pattern = 0
            self._initialize_patterns()

    def update(self, dt: float = 0.05):
        """Update robot positions using current Hopfield pattern"""
        # Ensure patterns are initialized
        if not hasattr(self.hopfield, 'velocity_patterns') or len(self.hopfield.velocity_patterns) == 0:
            self._initialize_patterns()
            
        if self.current_pattern >= len(self.hopfield.velocity_patterns):
            print(f"[WARNING] Invalid pattern index {self.current_pattern}, resetting to 0")
            self.current_pattern = 0
            
        # Get the pattern from Hopfield control
        pattern = self.hopfield.velocity_patterns[self.current_pattern]
        
        # Update each robot's velocity based on its chunk of the pattern
        for i, robot in enumerate(self.robots):
            # Extract the 4 neurons for this robot
            start_idx = i * 4
            neuron_chunk = pattern[start_idx:start_idx + 4]
            
            # Get swarm center and angular velocity
            center = np.mean([r.position for r in self.robots], axis=0)
            angular_velocity = 0.1 * (1 if self.current_pattern == 1 else -1)
            
            # Set velocity using rigid body transformation
            robot.set_velocity_from_chunk(
                neuron_chunk,
                center_position=center,
                angular_velocity=angular_velocity
            )
            
            # Update position
            robot.update_position(dt)

    def get_positions(self) -> np.ndarray:
        """Get current positions of all robots"""
        return np.array([robot.position for robot in self.robots])

    def recall_pattern(self, input_pattern, max_iter=10):
        """Recall closest stored pattern using Hopfield network"""
        return self.hopfield.recall_pattern(input_pattern, max_iter)

    def save_patterns(self, filename: str):
        """Save current velocity patterns to file"""
        np.savez(filename,
                 patterns=self.hopfield.velocity_patterns,
                 encodings=self.hopfield.encoded_patterns)
                 
    def load_patterns(self, filename: str):
        """Load velocity patterns from file"""
        data = np.load(filename)
        self.hopfield.velocity_patterns = data['patterns']
        self.hopfield.encoded_patterns = data['encodings']
        self.hopfield.hopfield_weights = self.hopfield.train_hopfield_network()

    def _initialize_patterns(self):
        """Initialize movement patterns for left and right turns"""
        filename = "../ui/pattern.npz"
        try:
            self.load_patterns(filename)
            # Verify loaded pattern matches robot count
            expected_neurons = 4 * len(self.robots)
            actual_neurons = self.hopfield.velocity_patterns.shape[1]
            if actual_neurons != expected_neurons:
                print(f"[WARNING] Loaded patterns have {actual_neurons} neurons, expected {expected_neurons}. Regenerating.")
                raise ValueError("Pattern size mismatch")
        except (FileNotFoundError, KeyError, ValueError) as e:
            print(f"[INFO] Generating new patterns: {str(e)}")
            # Re-initialize Hopfield patterns
            self.hopfield._initialize_patterns()
            self.save_patterns(filename)
            print(f"[INFO] New patterns saved for {len(self.robots)} robots")
