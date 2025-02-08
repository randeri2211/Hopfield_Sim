import numpy as np
from scipy.special import expit


class SwarmHopfieldControl:
    """Enhanced Hopfield network integration for swarm control with angular velocity encoding"""

    def __init__(self, robot_positions, speed=0.2, angular_speed=0.1):
        self.robot_positions = np.array(robot_positions)
        self.num_robots = len(robot_positions)
        print(f"[INIT] Creating Hopfield control with {self.num_robots} robots")
        print(f"[DEBUG] Expected pattern size: {4 * self.num_robots} neurons (4 neurons per robot)")
        print(f"[DEBUG] Using speed={speed}, angular_speed={angular_speed}")
        self.speed = speed
        self.angular_speed = angular_speed
        self._initialize_patterns()

    def _initialize_patterns(self):
        """Initialize patterns for left and right turns"""
        print("[DEBUG] Initializing patterns")
        self.velocity_patterns = self.generate_velocity_patterns()
        print(f"[DEBUG] Generated velocity patterns shape: {self.velocity_patterns.shape}")

        expected_size = 4 * self.num_robots
        if self.velocity_patterns.shape[1] != expected_size:
            raise ValueError(f"Pattern dimension mismatch: expected {expected_size}, got {self.velocity_patterns.shape[1]}")

        self.encoded_patterns = self.encode_patterns()
        print(f"[DEBUG] Encoded patterns shape: {self.encoded_patterns.shape}")
        print("[DEBUG] Left turn pattern:", self.encoded_patterns[0])
        print("[DEBUG] Right turn pattern:", self.encoded_patterns[1])

        self.hopfield_weights = self.train_hopfield_network()

    def generate_velocity_patterns(self):
        """Generate velocity patterns for left and right turns with 4 neurons per robot."""
        num_neurons = 4 * self.num_robots
        print(f"[DEBUG] Generating patterns for {self.num_robots} robots ({num_neurons} neurons)")

        # Create patterns for left/right turns and straight movement
        left_pattern = np.array([1, -1, 1, -1] * self.num_robots)  # Forward + left angular
        right_pattern = np.array([1, -1, -1, 1] * self.num_robots)  # Forward + right angular
        straight_pattern = np.array([1, -1, -1, -1] * self.num_robots)  # Forward only

        # Stack patterns into a 2D array
        patterns = np.vstack([left_pattern, right_pattern])

        print(f"[DEBUG] Generated patterns shape: {patterns.shape}")
        print("[DEBUG] Left turn pattern:", patterns[0])
        print("[DEBUG] Right turn pattern:", patterns[1])

        # Verify pattern dimensions
        if patterns.shape != (2, num_neurons):
            raise ValueError(f"Invalid pattern shape: expected (2, {num_neurons}), got {patterns.shape}")

        return patterns

    def encode_patterns(self):
        """Patterns are already encoded using -1 and 1."""
        patterns = self.velocity_patterns.copy()
        print("[DEBUG] Verifying pattern values are -1 or 1")
        for i, pattern in enumerate(patterns):
            for j, value in enumerate(pattern):
                if value not in [-1, 1]:
                    raise ValueError(f"Invalid pattern value {value} at pattern {i}, position {j}")
        return patterns

    def train_hopfield_network(self):
        """Train the Hopfield network using Hebbian learning rule."""
        pattern_size = 4 * self.num_robots  # 4 neurons per robot
        weights = np.zeros((pattern_size, pattern_size))
        print("[DEBUG] Training Hopfield network with pattern_size:", pattern_size)

        for pattern in self.encoded_patterns:
            weights += np.outer(pattern, pattern)

        np.fill_diagonal(weights, 0)
        return weights / len(self.encoded_patterns)

    def generate_default_patterns(self, rows, cols, num_directions):
        """Create basic directional movement patterns for swarm initialization"""
        self.direction_angles = np.linspace(0, 360, num_directions, endpoint=False)
        self.velocity_patterns = self.generate_velocity_patterns()
        self.encoded_patterns = self.encode_patterns()
        self.hopfield_weights = self.train_hopfield_network()

    def recall_pattern(self, input_pattern, max_iter=10):
        """Recall the closest pattern using the Hopfield network."""
        pattern = np.array(input_pattern)
        if pattern.shape[0] != self.hopfield_weights.shape[0]:
            raise ValueError(
                f"Pattern dimension mismatch: Expected {self.hopfield_weights.shape[0]}, "
                f"got {pattern.shape[0]}. Verify robot count matches Hopfield network initialization."
            )

        print(f"[DEBUG] Recalling pattern with dim {pattern.shape}, weights dim {self.hopfield_weights.shape}")

        for _ in range(max_iter):
            pattern = np.sign(self.hopfield_weights @ pattern)
        return pattern

    def get_velocity_from_binary(self, encoded_pattern):
        """Convert 4-neuron encoding to robot velocities."""
        print(f"[DEBUG] Processing pattern of shape: {encoded_pattern.shape}")

        # Find closest matching pattern
        similarities = [np.sum(encoded_pattern == encoded) for encoded in self.encoded_patterns]
        best_match_idx = np.argmax(similarities)
        pattern = self.encoded_patterns[best_match_idx]
        print(f"[DEBUG] Best matching pattern index: {best_match_idx}")

        # Convert pattern to velocities
        velocities = []
        for i in range(0, len(pattern), 4):
            neuron_chunk = pattern[i:i + 4]
            print(f"[DEBUG] Processing neuron chunk {i // 4}: {neuron_chunk}")

            # Forward/backward velocity (first two neurons)
            forward = neuron_chunk[0]  # 1 for forward, -1 for not forward
            backward = neuron_chunk[1]  # 1 for backward, -1 for not backward
            vx = self.speed if forward > backward else -self.speed

            # Left/right velocity (last two neurons)
            left = neuron_chunk[2]  # 1 for left, -1 for not left
            right = neuron_chunk[3]  # 1 for right, -1 for not right
            vy = self.angular_speed if left > right else -self.angular_speed

            velocities.extend([vx, vy])
            print(f"[DEBUG] Robot {i // 4} velocities: vx={vx}, vy={vy}")

        result = np.array(velocities)
        print(f"[DEBUG] Generated velocities shape: {result.shape}")
        return result

    def infer_direction(self, partial_pattern):
        """Infer direction from partial pattern."""
        # Use Hopfield network for pattern completion
        completed = self.recall_pattern(partial_pattern)

        # Find best matching direction
        similarities = [np.sum(completed == encoded) for encoded in self.encoded_patterns]
        best_match_idx = np.argmax(similarities)
        return best_match_idx, self.direction_angles[best_match_idx]

    def assess_recall(self, input_pattern):
        """
        Assess the recall performance of the Hopfield network.

        Args:
            input_pattern: The input pattern to test.

        Returns:
            A similarity score between the recalled pattern and the closest stored pattern.
        """
        recalled_pattern = self.recall_pattern(input_pattern)
        similarities = [np.dot(recalled_pattern, encoded) / (np.linalg.norm(recalled_pattern) * np.linalg.norm(encoded))
                        for encoded in self.encoded_patterns]
        best_match_idx = np.argmax(similarities)
        similarity_score = similarities[best_match_idx]
        return similarity_score, best_match_idx

    def visualize_patterns(self):
        """Display stored patterns and network weights."""
        print("Velocity Patterns:")
        for idx, velocities in enumerate(self.velocity_patterns):
            print(f"Direction {'left' if idx == 0 else 'right'}: {velocities}")
