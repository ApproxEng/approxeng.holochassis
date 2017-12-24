from abc import ABCMeta, abstractmethod
from math import asin, pi, sqrt
from time import sleep

from approxeng.chassis import rotate_vector, Motion, DeadReckoning, rotate_point
from approxeng.chassis.dynamics import MotionLimit
from euclid import Vector2, Point2


class Waypoints:
    def __init__(self, drive, sleep_time=0.1, speed=200, turn_speed=pi, min_distance=10):
        self.finished = False
        self.sleep_time = sleep_time
        self.drive = drive
        self.speed = speed
        self.turn_speed = turn_speed
        self.min_distance = min_distance

    def follow(self, waypoints):
        if len(waypoints) == 0:
            return
        else:
            self.finished = False
            while not self.finished:
                sleep(self.sleep_time)

                def on_approach():
                    self.finished = True

                self.drive.drive_at_world(waypoints[0][0], waypoints[0][1], speed=self.speed, on_approach=on_approach)
        self.follow(waypoints[1:])


class Drive:
    """
    High level class to manage the robot's motion, aggregates the chassis, motors and a bit of planning logic.
    """

    __metaclass__ = ABCMeta

    def __init__(self, chassis, counts_per_revolution=1.0):
        """
        Create a new Drive instance

        :param chassis: 
            A :class:`approxeng.holochassis.chassis.HoloChassis` used to compute kinematics
        :param counts_per_revolution:
            Counts per revolution used by the dead reckoning code
        """
        self.chassis = chassis
        # Maximum translation speed in mm/s
        self.max_trn = chassis.get_max_translation_speed()
        # Maximum rotation speed in radians/2
        self.max_rot = chassis.get_max_rotation_speed()
        self.front = 0.0
        self.dead_reckoning = DeadReckoning(chassis=chassis, counts_per_revolution=counts_per_revolution)
        self.motion_limit = None

    def set_motion_limit(self, accel_time):
        """
        Set a motion limit, or remove an existing one. The limit fixes the maximum rate of change in the requested
        motion.

        :param accel_time: 
            Either None to set no limit, or a number of seconds which will set the minimum time required to go from
            a standing start to full speed in any component of the requested motion.
        """
        if accel_time is None:
            self.motion_limit = None
        else:
            self.motion_limit = MotionLimit(
                linear_acceleration_limit=self.max_trn / accel_time,
                angular_acceleration_limit=self.max_rot / accel_time)

    def set_motion(self, motion):
        """
        Set the motor speeds according to the supplied motion relative to the robot's front.

        :param motion: 
            A motion, in robot space and relative to self.front. Any motion limit defined will be applied
            to the supplied motion. If this is None nothing will be done.
        """
        if motion is None:
            return
        if self.front != 0.0:
            motion = Motion(translation=rotate_vector(motion.translation, self.front), rotation=motion.rotation)
        if self.motion_limit is not None:
            motion = self.motion_limit.limit_and_return(motion)
        self.set_wheel_speeds_from_motion(motion)

    def reset_dead_reckoning(self):
        """
        Reads encoder values, then resets the pose held by the dead reckoning module. We do this because otherwise
        any hardware implementations which track absolute position will lead to a huge incorrect reading for the first
        dead reckoning period after a startup.
        """
        self.update_dead_reckoning()
        self.dead_reckoning.reset()

    def estimated_pose(self):
        """
        Return the pose from the dead reckoning managed by this class. Convenience method so we don't have to do
        e.g. drive.dead_reckoning.pose all the time
        :return: 
        """
        return self.dead_reckoning.pose

    def drive_at(self, x, y, speed, turn_speed=pi, min_distance=10, on_approach=None):
        """
        Set and return a motion to get to a target specified relative to the robot's coordinate system. Note 
        that the 'front' is added when the motion is applied to the robot, so this implicitly is relative to that,
        with positive y axis in the direction of the robot's front.

        :param x: 
            X coordinate of the target in mm    
        :param y: 
            Y coordinate of the target in mm
        :param speed:
            Desired linear speed
        :param turn_speed:
            If a motion can't be calculated then turn on the spot instead, at turn_speed radians per second
        :param min_distance:
            If defined, and the target is closer, then stop
        :param on_approach:
            If defined, and min_distance is defined and satisfied, call this function when we hit min_distance
        :return: 
            The :class:`approxeng.holochassis.chassis.Motion` that was applied.
        """
        # Explicitly cast to floats in case we're not...
        x = float(x)
        y = float(y)
        speed = float(speed)
        turn_speed = float(turn_speed)
        min_distance = float(min_distance)

        if min_distance is not None and sqrt(x * x + y * y) < min_distance:
            motion = Motion(translation=Vector2(0, 0), rotation=0)
            if on_approach is not None:
                on_approach()
        else:
            if x == 0:
                # Straight ahead, avoid future division by zero!
                motion = Motion(translation=Vector2(0, speed), rotation=0)
            elif abs(y) < abs(x) or y <= 0:
                # Turn first without moving
                if x > 0:
                    motion = Motion(translation=Vector2(0, 0), rotation=turn_speed)
                else:
                    motion = Motion(translation=Vector2(0, 0), rotation=-turn_speed)
            else:
                radius = y * y / x
                # Angle is clockwise rotation
                angle = asin(x / y)
                arc_length = angle * radius
                print(x, y, angle, arc_length)
                motion = Motion(translation=Vector2(0, speed), rotation=angle * speed / arc_length)
        self.set_motion(motion)
        return motion

    def drive_at_world(self, x, y, speed, turn_speed=pi, min_distance=10, on_approach=None):
        """
        Similar to drive_at, but x and y are specified in world coordinates, and the method uses the dead reckoning
        logic to map from world to robot coordinates

        :param x: 
        :param y: 
        :param speed: 
        :param turn_speed: 
        :param min_distance: 
        :param on_approach:
        :return: 
        """
        p = self.dead_reckoning.pose.position
        target_point = Point2(x=x - p.x, y=y - p.y)
        target_point = rotate_point(target_point, -self.front - self.dead_reckoning.pose.orientation)
        return self.drive_at(x=target_point.x, y=target_point.y, speed=speed, turn_speed=turn_speed,
                             min_distance=min_distance, on_approach=on_approach)

    @abstractmethod
    def set_wheel_speeds_from_motion(self, motion):
        """
        Set wheel speeds based on a :class:`approxeng.holochassis.chassis.Motion` instance
        """
        pass

    @abstractmethod
    def update_dead_reckoning(self):
        """
        Read angles from the motors and use them to update the current dead reckoning pose. 

        :returns:
            A :class:`approxeng.holochassis.chassis.Pose` containing the current dead reckoning pose
        """
        pass
