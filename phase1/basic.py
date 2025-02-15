# from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# from time import sleep
# from math import pi
#
# client = RemoteAPIClient()
# sim = client.getObject('sim')
# sim.setStepping(True)
#
# FINAL_TIME = 100
# ROBOT_NUM = 4
# velocity = 10
# turn_speed = 2
#
# try:
#     sim.startSimulation()
#     for i in range(ROBOT_NUM):
#         turn_joint = sim.getObject(f'/Robot{i}/Revolute_joint')
#         angle = 0.75 * pi + pi / 2 * i
#         sim.setJointPosition(turn_joint, angle)
#     while (t := sim.getSimulationTime()) < FINAL_TIME:
#         direction = 0
#         # for i in range(ROBOT_NUM):
#         #     turn_joint = sim.getObject(f'/Robot{i}/Revolute_joint')
#         #     forward_joint = sim.getObject(f'/Robot{i}/Revolute_joint/Revolute_joint')
#         #     actual_speed = 0
#         #     if i == 0 or i == 3:
#         #         actual_speed = velocity + direction * turn_speed
#         #     else:
#         #         actual_speed = velocity - direction * turn_speed
#         #     sim.setJointTargetVelocity(turn_joint, 0)
#         #     angle = (i+1)*45
#         #     print(f'Robot{i}:{angle}')
#         #     sim.setJointPosition(turn_joint, angle)
#         #
#         #     sim.setJointTargetVelocity(forward_joint, actual_speed)
#         for i in range(ROBOT_NUM):
#             forward_joint = sim.getObject(f'/Robot{i}/Revolute_joint/Revolute_joint')
#             turn_joint = sim.getObject(f'/Robot{i}/Revolute_joint')
#             sim.setJointTargetVelocity(turn_joint, 0)
#             sim.setJointTargetVelocity(forward_joint, 10)
#
#         print(f'Simulation time: {t:.2f} [s]')
#         sim.step()
#     sim.stopSimulation()
# except:
#     sim.stopSimulation()
