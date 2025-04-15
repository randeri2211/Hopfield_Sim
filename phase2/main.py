import math

import numpy as np

from phase1.utils import Robot_Swarm, sim, simUI, window_handle
from phase2.hopfield import Hopfield
from time import sleep


if __name__ == '__main__':
    rows = 3
    cols = 3
    bit_size = 4
    timeout = 100
    hop = Hopfield(rows, cols, 0.05, bit_size)
    swarm = Robot_Swarm('/Robot0_0', rows, cols, 0.12)
    swarm.create_swarm()
    r = sim.getObject('/Robot0_0')
    print(f'max num {hop.max_num}')
    print(f'pattern {hop.patterns[0]}')

    speed_mat = hop.get_speed_mat()
    print(hop.neurons)
    print(speed_mat)
    swarm.set_all_velocities(speed_mat)
    try:
        sleep(0.1)
        sim.startSimulation()
        while True:
            t = sim.getSimulationTime()

            # Start processing
            robot_velocities = np.array(swarm.get_all_velocities()).flatten()
            print(f'robot vel {robot_velocities}')
            for i in range(len(robot_velocities)):
                robot_velocities[i] = robot_velocities[i] / hop.max_num


            # robot_velocities = list(map(lambda x: (x / abs(x) if x != 0 else 1) * max(x / hop.max_num, 1), robot_velocities))
            # print(f'robot vel {robot_velocities}')
            encoded_velocities = hop.encode_pattern(robot_velocities)
            print(f'encoded vel {encoded_velocities}')
            hop.neurons = np.array(encoded_velocities)
            hop.update()
            speed_mat = hop.get_speed_mat()
            print(f'result {speed_mat}')
            print(f'pattern speed {hop.get_pattern_speed(0)}')
            swarm.set_all_velocities(speed_mat)
            # End processing

            if t >= timeout:
                sim.pauseSimulation()
                print("breaking")
                break
            # print(f'Simulation time: {t:.2f} [s]')
            sim.step()
    except (Exception, KeyboardInterrupt) as e:
        print(e)
        simUI.destroy(str(window_handle))
        sim.stopSimulation()
