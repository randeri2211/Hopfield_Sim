from math import dist, pi, copysign
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import sys

client = RemoteAPIClient()
sim = client.getObject('sim')
DYN_LOCK_PARAM_ID = 2002

class Robot_Swarm:
    def __init__(self, base, rows, cols, spacing):
        self.base_name = base.split('00')[0].split('/')[1]
        print(f'Robot Name:{self.base_name}')
        self.base = sim.getObject(base)
        self.rows = rows
        self.cols = cols
        self.spacing = spacing
        self.distances = [[None for j in range(cols)]for i in range(rows)]

    def create_swarm(self, created=False):
        if not created:
            self.duplicate()
            self.link()
        self.calc_distance_center()

    def duplicate(self):
        start_x = 0.0
        start_y = 0.0

        for i in range(self.rows):
            for j in range(self.cols):
                if i == 0 and j == 0:
                    continue
                try:
                    duplicated_handles = sim.copyPasteObjects([self.base], 1)

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

                alias_name = f'{self.base_name}{i}{j}'
                try:
                    sim.setObjectAlias(new_robot_handle, alias_name)
                except Exception as e:
                    print(f"Failed to rename new robot copy ({i},{j}) to {alias_name}:", e)

        print("Done creating robot swarm.")

    def link(self):
        for i in range(self.rows):
            for j in range(self.cols):
                dummyB = sim.getObject(f"/{self.base_name}{i}{j}/dummyB")
                dummyR = sim.getObject(f"/{self.base_name}{i}{j}/dummyR")
                if i != 0:
                    otherL = sim.getObject(f"/{self.base_name}{i - 1}{j}/dummyL")
                    sim.setLinkDummy(dummyR, otherL)
                if j != 0:
                    otherF = sim.getObject(f"/{self.base_name}{i}{j - 1}/dummyF")
                    sim.setLinkDummy(dummyB, otherF)

    def move(self, forward, right):
        for i in range(self.rows):
            for j in range(self.cols):
                speed_joint = sim.getObject(f'/Robot{i}{j}/Turn_joint/Speed_joint')
                turn_joint = sim.getObject(f'/Robot{i}{j}/Turn_joint')
                # Stop turn joints for now
                sim.setJointInterval(turn_joint, False, [0.0, 0.0])
                sim.setJointTargetVelocity(turn_joint, 0)
                sim.setJointTargetPosition(turn_joint, 0)
                turn_speed = right * self.distances[i][j][0] * self.distances[i][j][1]
                sim.setJointTargetVelocity(speed_joint, forward + turn_speed)
                print(f'{sim.getJointTargetVelocity(speed_joint)}', end=',')
            print()

    def calc_distance_center(self):
        robot_size = 0.1     # 0.1 x 0.1
        center = (self.rows * robot_size / 2, self.cols * robot_size / 2)
        for i in range(self.rows):
            for j in range(self.cols):
                robot_center = (i * robot_size + robot_size / 2, j * robot_size + robot_size / 2)
                if center[1] - robot_center[1] == 0:
                    dir = 0
                else:
                    dir = (center[1] - robot_center[1]) / abs(center[1] - robot_center[1])
                self.distances[i][j] = (dist(center, robot_center), dir * -1)   # 0-distance 1-directionality


if __name__ == '__main__':
    swarm = Robot_Swarm('/Robot00', 3, 3, 0.12)
    swarm.create_swarm(True)
    swarm.move(0, 100)

    try:
        sim.startSimulation()
        while True:
            t = sim.getSimulationTime()
            print(f'Simulation time: {t:.2f} [s]')
            sim.step()
    except:
        sim.stopSimulation()
