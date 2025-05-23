import random
import time

import numpy as np
from math import dist, cos, atan2


class Hopfield:
    def __init__(self, rows, columns, wheel_size, bit_size):
        self.bit_size = bit_size                        # Bit size determines max speed variance
        self.max_num = 2 ** (self.bit_size - 1) - 1

        self.rows = rows
        self.cols = columns
        self.wheel_size = wheel_size
        self.distances = [[() for _ in range(self.cols)] for _ in range(self.rows)]
        self.patterns = []
        self.init_patterns()

        self.neurons = np.random.uniform(-1, 1, len(self.patterns[0]))  # Neuron for each robot
        self.weights = self.train_hopfield_network()
        # print(self.weights)

    def __calc_distance_center(self):
        """Calculates the self.distances matrix
            each element inside self.distances is a factor of distance from center, directionality(left and right) and the column distance to center
        """
        robot_size = 0.1  # 0.1 x 0.1
        center = (self.rows * robot_size / 2, self.cols * robot_size / 2)
        for i in range(self.rows):
            for j in range(self.cols):
                robot_center = (i * robot_size + robot_size / 2, j * robot_size + robot_size / 2)

                distance_to_center = dist(center, robot_center)

                if center[0] - robot_center[0] == 0:
                    direction = 0
                else:
                    direction = 1

                # column_distance_to_center = abs((i + 0.5) * robot_size - center[0])
                product = 1 / cos(atan2(robot_center[1] - center[1], robot_center[0] - center[0]))
                self.distances[i][j] = (distance_to_center
                                        * direction
                                        * product
                                        # * column_distance_to_center
                                        )

    def init_patterns(self):
        self.patterns = []
        self.__calc_distance_center()
        # self.__calculate_pattern(1, 0)  # Forward
        # self.__calculate_pattern(-1, 0)  # Backward
        self.__calculate_pattern(0, 1)  # Right
        # self.__calculate_pattern(0, -1)  # Left

        self.encode_patterns()

        # for row in self.patterns:
        # print(row)

    def __calculate_pattern(self, forward, right):
        """
        :param forward: Forward motion speed (meters/second)
        :param right: Right turn speed (degrees/second)
        """
        forward_speed_wheel = forward / (self.wheel_size / 2)  # Forward speed by wheel size
        mat = []
        for i in range(self.rows):
            temp = []
            for j in range(self.cols):
                turn_speed = right * self.distances[i][j]
                temp.append(forward_speed_wheel + turn_speed)
            mat.append(temp)
        self.patterns.append(self.norm_and_flatten_mat(mat))
        print(self.patterns)

    @staticmethod
    def norm_and_flatten_mat(mat):
        """
        :param mat: a 2d list to normalize
        :return: norm_flat_mat: normalized flattened(into 1D) list
        """
        # Flatten the matrix to find global min and max
        flat_list = [abs(item) for row in mat for item in row]
        max_val = max(flat_list)
        min_val = 0
        flat_list = [item for row in mat for item in row]
        # Define normalization function
        normalize = lambda x: (x - min_val) / (max_val - min_val) if max_val != min_val else 0

        # Apply normalization using map
        norm_flat_mat = list(map(normalize, flat_list))

        return norm_flat_mat

    def encode_pattern(self, pattern):
        encoded_pattern = []
        # print(pattern)
        for number in pattern:
            encoded_number = []
            number = round(number * self.max_num)  # "Normalize" to the bit scale
            num = format(number, f'0{self.bit_size - 1}b')  # Format the number into a binary string representation
            # print(number)
            # print(num)
            encoded_number += [-1] if num[0] == '-' else [1]  # Parity bit

            for _ in range(self.bit_size - 1 - len(num) - 1 * encoded_number[0] == 1):  # Padding with -1
                encoded_number += [-1]

            for digit in num[encoded_number[0] == -1:]:
                encoded_number += [-1] if digit == '0' else [1]  # Add encoded bits

            encoded_pattern += encoded_number
        return encoded_pattern

    def encode_patterns(self):
        for i in range(len(self.patterns)):
            self.patterns[i] = self.encode_pattern(self.patterns[i])  # Swap patterns with the binary encoded pattern

    def train_hopfield_network(self):
        """Train the Hopfield network using Hebbian learning rule."""
        pattern_size = len(self.patterns[0])
        weights = np.zeros((pattern_size, pattern_size))

        for pattern in self.patterns:
            outer = np.outer(pattern, pattern)
            weights += outer

        np.fill_diagonal(weights, 0)
        return weights / len(self.patterns)

    def update(self):
        pick = random.randint(0, len(self.patterns[0]) / self.bit_size - 1)  # Pick random neuron to update
        pick = pick * self.bit_size                                         # Start of the robot block

        for neuron in range(pick, pick + self.bit_size):
            weight = self.weights[neuron]
            val = np.dot(self.neurons, weight)
            val = min(val, 1)
            val = max(val, -1)
            self.neurons[neuron] = val

    def decode(self, r, c):
        """
        :param r: row
        :param c: column
        :return: string representation of the desired robot speed
        """
        data = self.neurons[(r * self.cols + c) * self.bit_size:(r * self.cols + c + 1) * self.bit_size]
        data_str = ""
        for digit in data:
            data_str += "0" if digit <= 0 else "1"
        return data_str

    def to_int(self, r, c):
        """
        :param r: row
        :param c: column
        :return: int representation of the desired robot speed
        """
        bit_str = self.decode(r, c)
        sign = -1 if bit_str[0] == "0" else 1
        num = sign * int(bit_str[1:], 2)
        return num

    def get_speed_mat(self):
        mat = []
        for i in range(self.rows):
            temp = []
            for j in range(self.cols):
                temp.append(self.to_int(i, j))
            mat.append(temp)
        return mat

    def get_pattern_speed(self, index):
        temp = []
        for i in range(0, len(self.patterns[index]), self.bit_size):
            data_str = ""
            for j in range(self.bit_size):
                data_str += "0" if self.patterns[index][i + j] == -1 else "1"

            sign = -1 if data_str[0] == "0" else 1
            num = sign * int(data_str[1:], 2)

            temp.append(num)
        return temp



if __name__ == '__main__':
    np.set_printoptions(suppress=True, precision=6)
    rows = 3
    cols = 3
    bit_size = 4
    hop = Hopfield(rows, cols, 0.05, bit_size)

    runs = hop.rows * hop.cols * 20
    start_time = time.time()
    for _ in range(runs):
        hop.update()
        print(f'neurons {hop.neurons}')
    end_time = time.time()
    run_time = end_time - start_time

    print(f'Pattern:\n{hop.patterns[0]}')
    print(f'Len:{len(hop.patterns[0])}')
    print(
        # list(
        hop.neurons
        # )
    )


    # for i in range(rows):
    #     for j in range(cols):
    #         d = hop.decode(i, j)
    #         print(d)
    #         print(hop.to_int(i, j))
    # print(hop.to_int(rows - 1, cols - 1))
#   variable starting point leads to different end patterns
