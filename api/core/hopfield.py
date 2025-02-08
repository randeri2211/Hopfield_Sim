import numpy as np


class HopfieldNetwork:
    def __init__(self, num_neurons, pattern_size=None):
        self.num_neurons = num_neurons
        self.weights = np.zeros((num_neurons, num_neurons))
        self.pattern_size = pattern_size

    def train(self, patterns):
        """
        Train the Hopfield network using the Hebbian learning rule.
        Patterns are now represented using -1 and 1.
        """
        if not patterns:
            return

        # Ensure pattern size is set
        if self.pattern_size is None:
            self.pattern_size = len(patterns[0])

        # Check if all patterns have the same size
        for p in patterns:
            if len(p) != self.pattern_size:
                raise ValueError(f"All patterns must have {self.pattern_size} elements. Got {len(p)}")

        # Initialize weights matrix based on num_neurons
        if self.weights.shape != (self.num_neurons, self.num_neurons):
            self.weights = np.zeros((self.num_neurons, self.num_neurons))

        # Hebbian learning rule with -1 and 1 representation
        for p in patterns:
            p_resized = np.pad(p, (0, self.num_neurons - self.pattern_size), 'constant', constant_values=-1)
            self.weights += np.outer(p_resized, p_resized)
        np.fill_diagonal(self.weights, 0)

    def recall(self, input_pattern, steps=5):
        """
        Recall a pattern from the network.
        The input pattern should be represented using -1 and 1.
        """
        # Pad the input pattern with -1 if it's smaller than num_neurons
        if len(input_pattern) < self.num_neurons:
            input_pattern = np.pad(input_pattern, (0, self.num_neurons - len(input_pattern)), 'constant', constant_values=-1)

        for _ in range(steps):
            input_pattern = np.sign(self.weights @ input_pattern)
        return input_pattern[:self.pattern_size]


def create_grid_positions(rows=5, cols=3):
    return [(2 * i - rows + 1, 2 * j - cols + 1)
            for i in range(rows) for j in range(cols)]

if __name__ == '__main__':
    pass