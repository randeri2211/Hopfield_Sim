import math
from time import sleep
from math import dist, cos, atan2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')
DYN_LOCK_PARAM_ID = 2002


class Robot_Swarm:
    def __init__(self, base, rows, cols, spacing, left_dummy='dummyL', right_dummy='dummyR', front_dummy='dummyF', back_dummy='dummyB'):
        """
        :param base: base robot name,should include 00 at the end
        :param rows: number of rows for the robotic grid
        :param cols: number of columns for the robotic grid
        :param spacing: spacing to create the robots with
        """
        self.base_name = base.split('0_0')[0].split('/')[1]
        print(f'Robot Name:{self.base_name}')
        self.base = sim.getObject(base)
        self.rows = rows
        self.cols = cols
        self.spacing = spacing
        self.distances = [[() for _ in range(cols)] for _ in range(rows)]
        self.front_dummy = front_dummy
        self.back_dummy = back_dummy
        self.right_dummy = right_dummy
        self.left_dummy = left_dummy
        self.swarm = None
        self.wheel_size = None

    def figure_wheel_size(self):
        wheel_handle = sim.getObject(f'{self.base_name}0_0/Turn_joint/Speed_joint/Wheel')
        self.wheel_size = sim.getShapeGeomInfo(wheel_handle)[2][0]  # Get the x dimension of the wheel shape

    def create_swarm(self):
        """Creates the swarm with the given params on init"""
        self.swarm = sim.getObject('/Swarm', {'noError': True})
        #  No Swarm
        if self.swarm < 0:
            # Create a new swarm
            self.swarm = sim.createDummy(0.01)
            sim.setObjectAlias(self.swarm, 'Swarm')
            sim.setObjectParent(self.base, self.swarm, True)
        else:
            self.cull()

        self.base_name = f'/Swarm/{self.base_name}'
        self.duplicate()
        self.link()
        self.calc_distance_center()
        self.figure_wheel_size()

    def cull(self):
        """Cull all objects in the swarm except the original"""
        try:
            index = 0
            robot = sim.getObjectChild(self.swarm, index)
            while robot != -1:
                robot_name = sim.getObjectAlias(robot)
                if f'{robot_name}' != f'{self.base_name}0_0':
                    sim.removeModel(robot)
                else:
                    index += 1

                robot = sim.getObjectChild(self.swarm, index)
        except Exception as e:
            print(f'Cull Exception {e}')
        print('Done Culling')

    def duplicate(self):
        """Duplicates the base robot according to params given on init"""
        start_x = 0.0
        start_y = 0.0

        for i in range(self.rows):
            for j in range(self.cols):
                if i == 0 and j == 0:
                    continue
                try:
                    duplicated_handles = sim.copyPasteObjects([self.base], 1)
                    sim.setObjectParent(duplicated_handles[0], self.swarm, True)
                    new_robot_handle = duplicated_handles[0]
                except Exception as e:
                    print("Failed to copy/paste Robot:", e)
                    continue

                x_pos = start_x + j * self.spacing
                y_pos = start_y + i * self.spacing
                z_pos = 0.15

                try:
                    sim.setObjectPosition(new_robot_handle, -1, [x_pos, y_pos, z_pos])
                except Exception as e:
                    print(f"Failed to set position for new robot copy ({i},{j}):", e)

                alias_name = f'{self.base_name.split("/Swarm/")[1]}{i}_{j}'
                try:
                    sim.setObjectAlias(new_robot_handle, alias_name)
                except Exception as e:
                    print(f"Failed to rename new robot copy ({i},{j}) to {alias_name}:", e)

        print("Done creating robot swarm.")

    def link(self):
        """Links all the swarm robots together via the dummies"""
        for i in range(self.rows):
            for j in range(self.cols):
                dummyB = sim.getObject(f"{self.base_name}{i}_{j}/{self.back_dummy}")
                dummyR = sim.getObject(f"{self.base_name}{i}_{j}/{self.right_dummy}")
                if i != 0:
                    otherL = sim.getObject(f"{self.base_name}{i - 1}_{j}/{self.left_dummy}")
                    sim.setLinkDummy(dummyR, otherL)
                if j != 0:
                    otherF = sim.getObject(f"{self.base_name}{i}_{j - 1}/{self.front_dummy}")
                    sim.setLinkDummy(dummyB, otherF)

    def move(self, forward, left, draw_text=False):
        """
        :param forward: Forward motion speed (meters/second)
        :param left: Left turn speed (degrees/second)
        :param draw_text: Boolean flag,whether to draw the text onto the robot(Very time-consuming!)
        """
        sim.pauseSimulation()
        forward_speed_wheel = forward / (self.wheel_size / 2)
        for i in range(self.rows):
            for j in range(self.cols):
                speed_joint = sim.getObject(f'{self.base_name}{i}_{j}/Turn_joint/Speed_joint')
                turn_joint = sim.getObject(f'{self.base_name}{i}_{j}/Turn_joint')

                # Stop turn joints for now
                sim.setJointInterval(turn_joint, False, [0.0, 0.0])

                turn_speed = left * self.distances[i][j]
                sim.setJointTargetVelocity(speed_joint, forward_speed_wheel + turn_speed)

                if draw_text:

                    parent = sim.getObject(f'{self.base_name}{i}_{j}')
                    index = 0
                    obj = sim.getObjectChild(parent, index)
                    while obj != -1:
                        if sim.getProperty(obj, 'dummyType', {'noError': True}) == 8:
                            obj2 = sim.getObjectChild(obj, 0)
                            sim.removeObject(obj2)
                            sim.removeObject(obj)
                        else:
                            index += 1

                        obj = sim.getObjectChild(parent, index)

                    joint_speed = sim.getJointTargetVelocity(speed_joint)

                    # Create a 3D text shape
                    textShapeHandle = sim.generateTextShape(f'{"minus" if joint_speed < 0 else ""}{joint_speed:.2f}', [1, 1, 1], 0.01, True)

                    # Parent the shape to the robot
                    sim.setObjectParent(textShapeHandle, parent, True)

                    # Position the text shape a bit above the robot
                    sim.setObjectPosition(textShapeHandle, parent, [0, 0, 0.026])

                    # Rotate the text 90° around the X axis relative to 'parent'
                    sim.setObjectOrientation(textShapeHandle, parent, [0, 0, 0])
        sim.startSimulation()

    def calc_distance_center(self):
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

                column_distance_to_center = abs((i + 0.5) * robot_size - center[0])
                product = 1 / cos(atan2(robot_center[1] - center[1], robot_center[0] - center[0]))
                self.distances[i][j] = (distance_to_center
                                        * direction
                                        * product
                                        # * column_distance_to_center
                                        )


if __name__ == '__main__':
    swarm = Robot_Swarm('/Robot0_0', 5, 5, 0.12)
    sim.stopSimulation()
    swarm.create_swarm()

    r = sim.getObject('/Robot0_0')
    start_yaw = None
    try:
        sleep(0.1)
        sim.startSimulation()
        swarm.move(0, 90, False)
        done = False
        while True:
            angles = sim.getObjectOrientation(r)
            yaw, _, _ = sim.alphaBetaGammaToYawPitchRoll(*angles)
            if not start_yaw:
                start_yaw = yaw
            print(f'yaw:{yaw}')
            t = sim.getSimulationTime()

            if t > 1 and not done:
                done = True
                # swarm.move(0, 0, False)
                sim.pauseSimulation()
                break
            print(f'Simulation time: {t:.2f} [s]')
            sim.step()
    except Exception as e:
        print(e)

        sim.stopSimulation()
