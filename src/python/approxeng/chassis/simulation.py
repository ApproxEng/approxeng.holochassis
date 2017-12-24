from threading import Thread
from time import time, sleep

from approxeng.chassis import Motion, Pose
from approxeng.chassis.drive import Drive
from euclid import Vector2, Point2


class SimulationDrive(Drive):
    """
    Implementation of Drive which pushes Motion data into a Simulation and reads back its dead reckoning pose
    each time the simulation updates.
    """

    def __init__(self, chassis, simulation):
        super(SimulationDrive, self).__init__(chassis=chassis, )
        self.simulation = simulation
        self.simulation.after_simulation_callback = self.update_pose_from_simulation

    def update_pose_from_simulation(self):
        self.dead_reckoning.set_orientation(self.simulation.pose.orientation)
        self.dead_reckoning.set_position(self.simulation.pose.position)

    def set_wheel_speeds_from_motion(self, motion):
        self.simulation.motion = motion

    def update_dead_reckoning(self):
        pass


class Simulation(Thread):
    """
    A class which can model (in a strictly theoretical fashion) the movement of a chassis as specified by
    a :class:`approxeng.holochassis.chassis.Motion` instance. When running, the motion property is used to
    mutate a pose property which can then be read.
    """

    def __init__(self, name='SimulationThread', update_delay=0.01):
        super(Simulation, self).__init__(name=name)
        self.daemon = True
        self.motion = Motion(Vector2(0, 0), 0)
        self.pose = Pose(Point2(0, 0), 0)
        self.last_update_time = time()
        self.update_delay = update_delay
        self.after_simulation_callback = None
        self.running = True

    def set_motion(self, motion):
        self.motion = motion

    def run(self):
        print('Dynamics simulation starting')
        while self.running:
            now = time()
            time_delta = now - self.last_update_time
            self.last_update_time = now
            self.pose = self.pose.calculate_pose_change(motion=self.motion, time_delta=time_delta)
            if self.after_simulation_callback is not None:
                self.after_simulation_callback()
            sleep(self.update_delay)
        print('Dynamics simulation stopping')

    def stop(self):
        self.running = False
