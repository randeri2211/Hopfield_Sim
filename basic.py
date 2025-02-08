from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(True)

FINAL_TIME = 100
ROBOT_NUM = 4
velocity = 10
turn_speed = velocity / 4


try:
    sim.startSimulation()
    while (t := sim.getSimulationTime()) < FINAL_TIME:
        direction = 1
        for i in range(ROBOT_NUM):
            cuboid = sim.getObject(f'/Robot{i}/Revolute_joint')
            actual_speed = 0
            if i == 0 or i == 3:
                actual_speed = velocity + direction * turn_speed
            else:
                actual_speed = velocity - direction * turn_speed
            sim.setJointTargetVelocity(cuboid, actual_speed)
        print(f'Simulation time: {t:.2f} [s]')
        sim.step()
    sim.stopSimulation()
except:
    sim.stopSimulation()
