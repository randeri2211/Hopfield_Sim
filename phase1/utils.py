from time import sleep
from math import dist, pi
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

    def create_swarm(self):
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

    def cull(self):
        try:
            index = 0
            robot = sim.getObjectChild(self.swarm, index)
            while robot != -1:
                robot_name = sim.getObjectAlias(robot)
                if f'{robot_name}' != f'{self.base_name}0_0':
                    print(f'Culling {robot_name}')
                    sim.removeModel(robot)
                else:
                    index += 1

                robot = sim.getObjectChild(self.swarm, index)
        except Exception as e:
            print(f'Cull Exception {e}')
        print('Done Culling')

    def duplicate(self):
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
        sim.pauseSimulation()
        turn_speed_scalar = 60.5 / 9  # Constant for calibrating speed to match degrees per seconds(sort of)
        for i in range(self.rows):
            for j in range(self.cols):
                speed_joint = sim.getObject(f'{self.base_name}{i}_{j}/Turn_joint/Speed_joint')
                turn_joint = sim.getObject(f'{self.base_name}{i}_{j}/Turn_joint')
                # Stop turn joints for now
                sim.setJointInterval(turn_joint, False, [0.0, 0.0])

                turn_speed = left * self.distances[i][j][0] * self.distances[i][j][1] * self.distances[i][j][2] * turn_speed_scalar
                sim.setJointTargetVelocity(speed_joint, forward + turn_speed)
                joint_speed = f'{sim.getJointTargetVelocity(speed_joint):.2f}'

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

                    # Create a 3D text shape
                    textShapeHandle = sim.generateTextShape(joint_speed, [1, 1, 1], 0.01)

                    # Parent the shape to the robot
                    sim.setObjectParent(textShapeHandle, parent, True)

                    # Position the text shape a bit above the robot
                    sim.setObjectPosition(textShapeHandle, parent, [0, 0, 0.026])

                    # Rotate the text 90° around the X axis relative to 'parent'
                    sim.setObjectOrientation(textShapeHandle, parent, [0, 0, 0])
        sim.startSimulation()

    def calc_distance_center(self):
        robot_size = 0.1  # 0.1 x 0.1
        center = (self.rows * robot_size / 2, self.cols * robot_size / 2)
        for i in range(self.rows):
            for j in range(self.cols):
                robot_center = (i * robot_size + robot_size / 2, j * robot_size + robot_size / 2)
                if center[0] - robot_center[0] == 0:
                    direction = 0
                else:
                    direction = (center[0] - robot_center[0]) / abs(center[0] - robot_center[0])

                self.distances[i][j] = (dist(center, robot_center), direction, abs((i + 0.5) * robot_size - center[0]))  # 0-distance 1-directionality, 2-distance from center
                # print(self.distances[i][j][2])


if __name__ == '__main__':
    swarm = Robot_Swarm('/Robot0_0', 4, 4, 0.12)
    sim.stopSimulation()
    swarm.create_swarm()

    r = sim.getObject('/Robot0_0')
    start_yaw = None
    try:
        sleep(0.1)
        sim.startSimulation()
        swarm.move(0, -10)
        done = False
        while True:
            angles = sim.getObjectOrientation(r)
            yaw, _, _ = sim.alphaBetaGammaToYawPitchRoll(*angles)
            if not start_yaw:
                start_yaw = yaw
            print(f'yaw:{yaw}')
            t = sim.getSimulationTime()

            if t > 2 and not done:
                done = True
                swarm.move(0, 10, True)

            print(f'Simulation time: {t:.2f} [s]')
            sim.step()
    except:
        sim.stopSimulation()
