from time import sleep
from math import dist, cos, atan2, pi
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

MAX_SPEED = 5
MAX_TURN = 90
INC = 100
DRAW_TEXT = False

swarm = None
client = RemoteAPIClient()
client.require('simUI')
sim = client.getObject('sim')
simUI = client.getObject('simUI')
sim.setStepping(True)

last_forward = 0
last_right = 0

xml = f"""
<ui title="Python UI" closeable="true" resizable="true" activate="false" onclose="">
    <label id="1" text="Forward: {last_forward}\nRight: {last_right}" wordwrap="true"/> 
    <vslider on-change="forward_click" value="{last_forward}" maximum="{MAX_SPEED * INC}" minimum="{-MAX_SPEED * INC}"/>
    <hslider on-change="right_click" value="{last_right}" maximum="{MAX_TURN}" minimum="{-MAX_TURN}"/>
</ui>
"""
window_handle = simUI.create(xml)
screen_height = 950
ui_width, ui_height = simUI.getSize(window_handle)
new_loc = screen_height - ui_height
simUI.setPosition(window_handle, 45, new_loc)


def forward_click(arg1, button_id, val):
    global last_forward
    last_forward = val / INC
    swarm.move(last_forward, last_right, DRAW_TEXT)
    simUI.setLabelText(window_handle, 1, f"Forward: {last_forward}\nRight: {last_right}")


def right_click(arg1, button_id, val):
    global last_right
    last_right = val
    swarm.move(last_forward, last_right, DRAW_TEXT)
    simUI.setLabelText(window_handle, 1, f"Forward: {last_forward}\nRight: {last_right}")


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
        self.wheel_size = None      # Wheel Diameter

    def figure_wheel_size(self):
        wheel_handle = sim.getObject(f'{self.base_name}0_0/Turn_joint/Speed_joint/Wheel')
        self.wheel_size = sim.getShapeGeomInfo(wheel_handle)[2][0]  # Get the x dimension of the wheel shape (wheel diameter)

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
                z_pos = 0.05

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

    def move(self, forward, right, draw_text=False):
        """
        :param forward: Forward motion speed (meters/second)
        :param right: Right turn speed (degrees/second)
        :param draw_text: Boolean flag,whether to draw the text onto the robot(Very time-consuming!)
        """
        if draw_text:
            sim.pauseSimulation()
        forward_speed_wheel = forward / (self.wheel_size / 2)   # Forward speed by wheel size
        mat = []
        for i in range(self.rows):
            temp = []
            for j in range(self.cols):
                speed_joint = sim.getObject(f'{self.base_name}{i}_{j}/Turn_joint/Speed_joint')
                turn_joint = sim.getObject(f'{self.base_name}{i}_{j}/Turn_joint')

                # Stop turn joints for now
                sim.setJointInterval(turn_joint, False, [0.0, 0.0])

                turn_speed = right * self.distances[i][j]
                sim.setJointTargetVelocity(speed_joint, forward_speed_wheel + turn_speed)
                temp.append(forward_speed_wheel + turn_speed)

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
            mat.append(temp)
        # print(self.normalize_2d_list(mat))
        sim.startSimulation()

    @staticmethod
    def normalize_2d_list(matrix):
        # Flatten the matrix to find global min and max
        flat_list = [abs(item) for row in matrix for item in row]
        max_val = max(flat_list)
        min_val = 0
        # Define normalization function
        normalize = lambda x: (x - min_val) / (max_val - min_val) if max_val != min_val else 0

        # Apply normalization using map
        normalized_matrix = list(map(lambda row: list(map(normalize, row)), matrix))

        return normalized_matrix

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

                # column_distance_to_center = abs((i + 0.5) * robot_size - center[0])
                product = 1 / cos(atan2(robot_center[1] - center[1], robot_center[0] - center[0]))
                self.distances[i][j] = (distance_to_center
                                        * direction
                                        * product
                                        # * column_distance_to_center
                                        )

    def get_robot_velocity(self, i, j):
        speed_joint = sim.getObject(f'{self.base_name}{i}_{j}/Turn_joint/Speed_joint')
        return sim.getJointTargetVelocity(speed_joint)

    def get_all_velocities(self):
        vel_mat = []
        for i in range(self.rows):
            temp = []
            for j in range(self.cols):
                temp.append(self.get_robot_velocity(i, j))
            vel_mat.append(temp)
        return vel_mat

    def set_robot_velocity(self, i, j, velocity):
        speed_joint = sim.getObject(f'{self.base_name}{i}_{j}/Turn_joint/Speed_joint')
        sim.setJointTargetVelocity(speed_joint, velocity)

    def set_all_velocities(self, vel_mat):
        for i in range(self.rows):
            for j in range(self.cols):
                self.set_robot_velocity(i, j, vel_mat[i][j])


if __name__ == '__main__':
    swarm = Robot_Swarm('/Robot0_0', 3, 3, 0.12)
    sim.stopSimulation()
    swarm.create_swarm()

    r = sim.getObject('/Robot0_0')
    start_yaw = None
    try:
        sleep(0.1)
        swarm.move(0, 90, True)
        sim.startSimulation()
        done = False
        while True:
            angles = sim.getObjectOrientation(r)
            yaw, _, _ = sim.alphaBetaGammaToYawPitchRoll(*angles)
            yaw = yaw * 180 / pi
            if not start_yaw:
                start_yaw = yaw
            # print(f'yaw:{yaw}')
            t = sim.getSimulationTime()

            # if t >= 5 and not done:
            #     done = True
            #     # swarm.move(0, 0, False)
            #     sim.pauseSimulation()
            #     break
            # print(f'Simulation time: {t:.2f} [s]')
            sim.step()
    except:
        simUI.destroy(str(window_handle))
        sim.stopSimulation()
