from math import pi

from approxeng.chassis.util import get_regular_triangular_chassis
from approxeng.chassis.drive import Waypoints
from approxeng.chassis.simulation import Simulation, SimulationDrive
from approxeng.chassis.simulationplot import show_plot

# On MacOS you might need to do echo "backend: TkAgg" >> ~/.matplotlib/matplotlibrc to get this to work and avoid the
# 'Not installed as a framework' you otherwise pick up from the brew based installation of Python3

simulation = Simulation()
drive = SimulationDrive(simulation=simulation,
                        chassis=get_regular_triangular_chassis(wheel_distance=160,
                                                               wheel_radius=60,
                                                               max_rotations_per_second=500 / 60))
drive.set_motion_limit(4.0)
drive.front = pi / 2
show_plot(simulation).start()
simulation.start()
Waypoints(drive=drive, speed=200, turn_speed=pi).follow([(300, 600), (-200, -200), (0, 350), (100, 500)])
print('Waypoint set completed')
simulation.stop()
