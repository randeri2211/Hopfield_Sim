from typing import Tuple
import numpy as np


class DifferentialRobot:
    """Represents a differential drive robot with position and velocity control"""

    def __init__(self, robot_id: int, initial_position: Tuple[float, float] = (0.0, 0.0)):
        self.robot_id = robot_id
        self.position = np.array(initial_position, dtype=np.float64)
        self.velocity = np.array([0.0, 0.0], dtype=np.float64)  # [vx, vy]
        self.max_speed = 1.0  # meters/second

    # def move(self, velocity: Tuple[float, float], delta_time: float):
    #     """Move the robot based on velocity components"""
    #     if not isinstance(velocity, (tuple, list)) or len(velocity) != 2:
    #         print(f"[WARNING] Invalid velocity format: {velocity}")
    #         return
    #
    #     vx, vy = velocity
    #     if not (isinstance(vx, (int, float)) and isinstance(vy, (int, float))):
    #         print(f"[WARNING] Invalid velocity values: vx={vx}, vy={vy}")
    #         return
    #
    #     print(f"[DEBUG] Robot {self.robot_id} moving with vx={vx}, vy={vy}")
    #     self.set_velocity(float(vx), float(vy))
    #     self.update_position(delta_time)
    #     print(f"[DEBUG] Robot {self.robot_id} new position: {self.position}")

    def update_position(self, delta_time: float):
        """Update position based on current velocity and time delta"""
        movement = self.velocity * delta_time
        print(f"[DEBUG] Robot {self.robot_id} movement: {movement}")
        self.position += movement

    def set_velocity(self, vx: float, vy: float):
        """Set velocity vector with speed limiting"""
        target_velocity = np.array([vx, vy], dtype=np.float64)
        speed = np.linalg.norm(target_velocity)
        print(f"[DEBUG] Robot {self.robot_id} target speed: {speed}")

        if speed > self.max_speed:
            target_velocity = target_velocity / speed * self.max_speed
            print(f"[DEBUG] Robot {self.robot_id} speed limited to {self.max_speed}")

        self.velocity = target_velocity
        print(f"[DEBUG] Robot {self.robot_id} final velocity: {self.velocity}")

    def set_velocity_from_chunk(self, velocity_chunk, center_position, angular_velocity):
        """Set velocity using rigid body transformation around swarm center
        Args:
            velocity_chunk: 4-neuron pattern [surge, sway, heave, yaw]
            center_position: Current swarm center (x,y)
            angular_velocity: Angular velocity of swarm (radians/sec)
        
        Neurons represent: [forward, backward, left, right]
        Values are -1 or 1
        """
        if len(velocity_chunk) != 4:
            raise ValueError(f"Expected 4 neurons, got {len(velocity_chunk)}")

        print(f"[DEBUG] Robot {self.robot_id} processing neurons: {velocity_chunk}")

        # Calculate forward/backward velocity (first two neurons)
        forward = float(velocity_chunk[0])  # 1 for forward, -1 for not forward
        backward = float(velocity_chunk[1])  # 1 for backward, -1 for not backward

        # Calculate left/right velocity (last two neurons)
        left = float(velocity_chunk[2])  # 1 for left, -1 for not left
        right = float(velocity_chunk[3])  # 1 for right, -1 for not right

        # Calculate swarm-centric velocity components
        linear_speed = self.max_speed * 0.5 * (forward - backward)
        angular_speed = self.max_speed * 0.25 * (right - left)

        # Get position relative to swarm center
        r = self.position - center_position
        # Calculate tangential velocity from angular movement
        tangential_velocity = np.array([
            -angular_speed * r[1],
            angular_speed * r[0]
        ])

        # Combine linear and angular components
        vx = linear_speed + tangential_velocity[0]
        vy = tangential_velocity[1]
        print(vx)
        # Set final velocity with speed limiting
        self.set_velocity(vx, vy)

    def get_state(self) -> dict:
        """Return current state as dictionary"""
        return {
            'id': self.robot_id,
            'position': self.position.copy(),
            'velocity': self.velocity.copy()
        }

    def get_velocity(self) -> np.ndarray:
        """Return current velocity vector"""
        return self.velocity.copy()

    def __repr__(self) -> str:
        return f"DifferentialRobot(id={self.robot_id}, position={self.position}, velocity={self.velocity})"

#
# class DifferentialRobot:
#     """Represents a differential drive robot with position and velocity control"""
#
#     def __init__(self, robot_id: int, initial_position: Tuple[float, float] = (0.0, 0.0)):
#         self.robot_id = robot_id
#         self.position = np.array(initial_position, dtype=np.float64)
#         self.velocity = np.array([0.0, 0.0], dtype=np.float64)  # [vx, vy]
#         self.max_speed = 1.0  # meters/second
#
#     def move(self, velocity: Tuple[float, float], delta_time: float):
#         """Move the robot based on velocity components"""
#         if not isinstance(velocity, (tuple, list)) or len(velocity) != 2:
#             print(f"[WARNING] Invalid velocity format: {velocity}")
#             return
#
#         vx, vy = velocity
#         if not (isinstance(vx, (int, float)) and isinstance(vy, (int, float))):
#             print(f"[WARNING] Invalid velocity values: vx={vx}, vy={vy}")
#             return
#
#         print(f"[DEBUG] Robot {self.robot_id} moving with vx={vx}, vy={vy}")
#         self.set_velocity(float(vx), float(vy))
#         self.update_position(delta_time)
#         print(f"[DEBUG] Robot {self.robot_id} new position: {self.position}")
#
#     def update_position(self, delta_time: float):
#         """Update position based on current velocity and time delta"""
#         movement = self.velocity * delta_time
#         print(f"[DEBUG] Robot {self.robot_id} movement: {movement}")
#         self.position += movement
#
#     def set_velocity(self, vx: float, vy: float):
#         """Set velocity vector with speed limiting"""
#         target_velocity = np.array([vx, vy], dtype=np.float64)
#         speed = np.linalg.norm(target_velocity)
#         print(f"[DEBUG] Robot {self.robot_id} target speed: {speed}")
#
#         if speed > self.max_speed:
#             target_velocity = target_velocity / speed * self.max_speed
#             print(f"[DEBUG] Robot {self.robot_id} speed limited to {self.max_speed}")
#
#         self.velocity = target_velocity
#         print(f"[DEBUG] Robot {self.robot_id} final velocity: {self.velocity}")
#
#     def set_velocity_from_chunk(self, velocity_chunk, center_position, angular_velocity):
#         """Set velocity using rigid body transformation around swarm center
#         Args:
#             velocity_chunk: 4-neuron pattern [surge, sway, heave, yaw]
#             center_position: Current swarm center (x,y)
#             angular_velocity: Angular velocity of swarm (radians/sec)
#
#         Neurons represent: [forward, backward, left, right]
#         Values are -1 or 1
#         """
#         if len(velocity_chunk) != 4:
#             raise ValueError(f"Expected 4 neurons, got {len(velocity_chunk)}")
#
#         print(f"[DEBUG] Robot {self.robot_id} processing neurons: {velocity_chunk}")
#
#         # Calculate forward/backward velocity (first two neurons)
#         forward = float(velocity_chunk[0])  # 1 for forward, -1 for not forward
#         backward = float(velocity_chunk[1])  # 1 for backward, -1 for not backward
#
#         # Calculate left/right velocity (last two neurons)
#         left = float(velocity_chunk[2])  # 1 for left, -1 for not left
#         right = float(velocity_chunk[3])  # 1 for right, -1 for not right
#
#         # Calculate swarm-centric velocity components
#         linear_speed = self.max_speed * 0.5 * (float(velocity_chunk[0]) - float(velocity_chunk[1]))
#         angular_speed = self.max_speed * 0.25 * (float(velocity_chunk[3]) - float(velocity_chunk[2]))
#
#         # Get position relative to swarm center
#         r = self.position - center_position
#
#         # Calculate tangential velocity from angular movement
#         tangential_velocity = np.array([-angular_speed * r[1], angular_speed * r[0]])
#
#         # Calculate total velocity as combination of linear and angular components
#         vx = linear_speed + tangential_velocity[0]
#         vy = tangential_velocity[1]  # Pure rotation in y-direction
#
#         print(f"[DEBUG] Robot {self.robot_id} calculated velocities: vx={vx:.2f}, vy={vy:.2f}")
# import numpy as np
# from typing import Tuple
