from math import pi

from approxeng.chassis.simulation import Simulation, SimulationDrive
from approxeng.chassis.simulationplot import show_plot
from approxeng.chassis.util import get_regular_triangular_chassis

simulation = Simulation()
drive = SimulationDrive(simulation=simulation,
                        chassis=get_regular_triangular_chassis(wheel_distance=204,
                                                               wheel_radius=29.5,
                                                               max_rotations_per_second=500 / 60))
show_plot(simulation).start()
simulation.start()
drive.drive_at(x=70, y=150, speed=200, turn_speed=pi)
while 1:
    pass
