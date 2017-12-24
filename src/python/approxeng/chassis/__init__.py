from abc import ABCMeta
from math import cos, sin, degrees, radians, pi
from time import time

from euclid import Vector2, Point2

try:
    # We only need numpy under some conditions, try to import it and mark whether we succeeded
    from numpy import array as np_array
    from numpy.linalg import solve as np_solve

    NUMPY_AVAILABLE = True
except ImportError:
    # If not this isn't a problem unless you've got more than three wheels and need the 'motion from wheel speeds'
    # functionality. If this is the case you should probably go and get numpy first.
    NUMPY_AVAILABLE = False

__author__ = 'tom'


def rotate_point(point, angle, origin=None):
    """
    Rotate a Point2 around another Point2

    :param euclid.Point2 point:
        The point to rotate
    :param float angle:
        Angle in radians, clockwise rotation
    :param euclid.Point2 origin:
        Origin of the rotation, defaults to (0,0) if not specified
    :return:
        A new :class:`euclid.Point2` containing the rotated input point
    """
    if origin is None:
        origin = Point2(0, 0)
    s = sin(-angle)
    c = cos(-angle)
    return Point2(c * (point.x - origin.x) - s * (point.y - origin.y) + origin.x,
                  s * (point.x - origin.x) + c * (point.y - origin.y) + origin.y)


def rotate_vector(vector, angle, origin=None):
    """
    Rotate a :class:`euclid.Vector2` around a :class:`euclid.Point2`

    :param euclid.Vector2 vector:
        The vector to rotate
    :param float angle:
        Angle in radians, clockwise rotation
    :param euclid.Point2 origin:
        Origin of the rotation, defaults to (0,0) if not specified
    :return:
        A new :class:`euclid.Point2` containing the rotated input point
    """
    if origin is None:
        origin = Point2(0, 0)
    s = sin(-angle)
    c = cos(-angle)
    return Vector2(c * (vector.x - origin.x) - s * (vector.y - origin.y) + origin.x,
                   s * (vector.x - origin.x) + c * (vector.y - origin.y) + origin.y)


def smallest_difference(a, b, max_value=2 * pi):
    """
    Given two floats, a and b, and a maximum possible value for both a and b, calculate the smallest delta from a to b.
    For example, if a=1.0, b=2.5 and max_value=2.6, this should return -1.1, as subtracting 1.1 from a would result in
    -0.1, which will then be transformed to 2.5 after taking its modulus with 2.6. If max_value was 10, it would return
    +1.5, as this is the lower magnitude delta needed to go from 1.0 to 2.5. This function is used when calculating the
    shortest delta between two pose orientations, for this reason the max_value defaults to 2*pi for use when working
    in radians.

    If either a or b are less than zero or greater than the maximum value they will be treated as a % max_value or b %
    max_value respectively for the purposes of this calculation.

    :param float a:
        First value (see above)
    :param b:
        Second value (see above)
    :param max_value:
        Modulus, defaults to 2*pi if not specified
    :return:
        A value d such that (a + d) % max_value == b, and abs(d) is minimal (as there would be an infinite number of
        possible d that satisfy this relationship).
    """
    mod_a = a % max_value
    mod_b = b % max_value
    if abs(mod_a - mod_b) <= max_value / 2:
        return mod_b - mod_a
    elif mod_a >= mod_b:
        return mod_b + (max_value - mod_a)
    else:
        return -(mod_a + (max_value - mod_b))


class WheelSpeeds:
    """
    A simple container to hold desired wheel speeds, and to indicate whether any speeds were scaled back due to
    impossibly high values.
    """

    def __init__(self, speeds, scaling):
        """
        Create a new wheel speeds container

        :param speeds:
            A sequence of float values, one per wheel, in revolutions per second
        :param float scaling:
            If a requested translation or rotation was too fast for the chassis to perform, it will return an instance
            of this class with the scaling set to a value greater than 1.0. This indicates that it was unable to
            provide the requested trajectory but has instead provided the highest magnitude one possible. This parameter
            then contains the proportion of the requested trajectory that was possible to provide. For example, if
            the motion requested was a translation of 10mm/s in the X axis and a rotation of 10 radians per second, but
            on calculation this resulted in excessive wheel speeds which weren't possible, it might be scaled back to
            6mm/s on X and 6 radians per second - the motion is proportionately the same just slower, and in this case
            the scaling value would be 0.6.
        """
        self.speeds = speeds
        self.scaling = scaling

    def __str__(self):
        return 'WheelSpeeds[ speeds={}, scaling={} ]'.format(self.speeds, self.scaling)


class Motion:
    """
    A container to hold the translation and rotation vector representing the robot's motion. This is always expressed
    in the robot's coordinate frame, so a translation component of 0,1 always means the robot is heading forwards,
    irrespective of the current orientation of the robot (i.e. if the robot was turned 90 degrees in world space this
    0,1 motion would be a movement along the X axis in world space, but the Y axis in robot space). The rotation
    component of the motion is expressed in radians per second, positive values corresponding to clockwise rotation
    when viewed from the direction relative to the plane such that X is positive to the right and Y positive upwards.
    """

    def __init__(self, translation=None, x=None, y=None, rotation=0):
        """
        Constructor

        :param euclid.Vector2 translation:
            Vector2 representing the translation component in robot coordinate space of the motion. Defaults to
            Vector2(0,0)
        :param float rotation:
            Rotation in radians per second. Defaults to 0.
        """
        if translation is not None:
            self.translation = translation
        elif x is not None and y is not None:
            self.translation = Vector2(x, y)
        else:
            self.translation = Vector2(0, 0)
        self.rotation = rotation

    def __str__(self):
        return 'Motion[ x={}, y={}, theta={} (deg={}) ]'.format(self.translation.x, self.translation.y, self.rotation,
                                                                degrees(self.rotation))


class DeadReckoning:
    """
    Encapsulates the logic required to track the robot's position in world space using wheel encoders and chassis
    kinematics. To update the state of this object you need to call the update_from_counts function - this will
    compute the difference in counts for each wheel, and from this derive the rotational speed for each wheel since
    the last measurement. The :class:`approxeng.chassis.HoloChassis` is then used to convert these speeds into an arc,
    with the assumption that wheel speeds were constant during the time interval. This arc is used to update the
    :class:`approxeng.chassis.Pose` representing the current best estimate of the robot's position.

    Because this is in effect integrating over sensor readings, any errors, particularly in the chassis geometry or
    dimensions, or in the number of counts per revolution (for example if the gearing isn't quite what you think it is
    or there's enough slop in the gearbox that readings can drift) will accumulate over time. To mitigate this, if you
    have precise instantaneous information such as a compass reading every few seconds, these readings can be used to
    explicitly set the position, orientation, or both of the :class:`approxeng.chassis.Pose` tracked by this class.

    As there's an implicit assumption that wheel speeds are constant between encoder readings, this class will yield
    more accurate results when updated frequently. The exact optimal update frequency will depend on the encoder
    resolutions, chassis geometry etc. Some manual tuning may be required.
    
    The 'counts per revolution' can be used even when a non-counter-based sensor is employed. For example, if your
    sensor returns the cumulative number of degrees as a floating point value, you can just specify that there are 360
    counts per revolution and set max_count_value to None
    """

    def __init__(self, chassis, counts_per_revolution=64 * 19, max_count_value=1 << 15):
        """
        Constructor

        :param approxeng.chassis.HoloChassis chassis:
            The :class:`approxeng.chassis.HoloChassis` to be used to define kinematics for this DeadReckoning
        :param float counts_per_revolution:
            The number of counts registered by the wheel encoders per revolution of the wheel. Defaults to 64*19 to
            be the 64 count encoder fitted to a 19:1 reduction gearbox.
        :param int max_count_value:
            The largest value read from the encoders, this is used to determine when we've wrapped around the zero
            point, defaults to 1<<16 to reflect that count values are held in the microcontroller module as a uint16_t
        """
        self.chassis = chassis
        self.counts_per_revolution = counts_per_revolution
        self.max_count_value = max_count_value
        self.last_encoder_values = None
        self.last_reading_time = None
        self.pose = Pose(Point2(0, 0), 0)

    def reset(self):
        """
        Clear the state of this :class:`approxeng.chassis.DeadReckoning`
        """
        self.last_encoder_values = None
        self.last_reading_time = None
        self.pose = Pose(Point2(0, 0), 0)

    def set_position(self, position):
        """
        Explicitly set the position of the robot in world coordinates. Overrides the current value tracked by this
        instance. Use this when you have better information and want to update the state accordingly.

        :param euclid.Point2 position:
            The new position to set, as a :class:`euclid.Point2`, coordinates are in mm
        """
        self.pose.position = position
        return self.pose

    def set_orientation(self, orientation):
        """
        Explicitly set the orientation of the robot in world coordinates. Use this to explicitly update the orientation,
        for example when you have a sufficiently accurate compass fix that it can be used to eliminate any accumulated
        errors built up by the dead reckoning algorithm.

        :param float orientation:
            The new orientation to set, in radians from the positive Y axis, clockwise rotations being positive. This
            value will be normalised to the range 0-2PI
        :return:
            The current (updated) value of the :class:`approxeng.chassis.Pose`
        """
        self.pose.orientation = orientation % (2 * pi)
        return self.pose

    def update_from_counts(self, counts):
        """
        Update the pose from a new set of encoder values

        :param counts:
            A list of encoder counts, one per wheel
        :return:
            The updated :class:`approxeng.chassis.Pose` object (this is also modified in the internal state of the
            DeadReckoning)
        """
        reading_time = time()
        if self.last_encoder_values is None:
            self.last_encoder_values = counts
            self.last_reading_time = reading_time
            self.pose = Pose(Point2(0, 0), 0)
        else:
            time_delta = reading_time - self.last_reading_time
            wheel_speeds = [smallest_difference(current_reading, last_reading, self.max_count_value) / (
                    self.counts_per_revolution * time_delta) for last_reading, current_reading
                            in zip(counts, self.last_encoder_values)]
            motion = self.chassis.calculate_motion(speeds=wheel_speeds)
            self.pose = self.pose.calculate_pose_change(motion, time_delta)
            self.last_encoder_values = counts
            self.last_reading_time = reading_time
        return self.pose

    def update_from_revolutions(self, counts):
        """
        Update the pose from calculated wheel revolutions
        
        :param counts: 
            A list of float values containing cumulative wheel angles in revolutions, i.e. if a wheel has rotated two
            turns clockwise this will be 2.0
        :return: 
        """
        reading_time = time()
        if self.last_encoder_values is None:
            self.last_encoder_values = counts
            self.last_reading_time = reading_time
            self.pose = Pose(Point2(0, 0), 0)
        else:
            time_delta = reading_time - self.last_reading_time
            wheel_speeds = [(current_reading - last_reading) / time_delta
                            for last_reading, current_reading in zip(counts, self.last_encoder_values)]
            motion = self.chassis.calculate_motion(speeds=wheel_speeds)
            self.pose = self.pose.calculate_pose_change(motion, time_delta)
            self.last_encoder_values = counts
            self.last_reading_time = reading_time
        return self.pose


class Pose:
    """
    A container to hold the position as a Point2 along with orientation in radians, where 0 corresponds to the positive
    Y axis (0,1). Orientation is expressed in radians, with positive values indicating a rotation from the positive Y
    axis in the clockwise direction, i.e. a rotation of 0 is North, pi/2 East, pi South and 3pi/2 West.
    """

    def __init__(self, position=None, orientation=0):
        """
        Constructor

        :param euclid.Point2 position:
            A Point2 containing the position of the centre of the robot. Defaults to Point2(0,0)
        :param float orientation:
            Orientation in radians, 0 being the positive Y axis, positive values correspond to clockwise rotations, i.e.
            pi/4 is East. This value will be normalised to be between 0 and 2 * pi. Defaults to 0
        """
        if position is not None:
            self.position = position
        else:
            self.position = Point2(0, 0)
        self.orientation = orientation % (2 * pi)

    def distance_to_pose(self, to_pose):
        """
        Return the distance to the other pose position

        :param approxeng.chassis.Pose to_pose:
            The target pose
        """
        return abs(self.position - to_pose.position)

    def is_close_to(self, to_pose, max_distance=0.001, max_orientation_difference=radians(1)):
        """
        Check whether we're close to the specified pose, defining closeness as both distance on the plane and difference
        in orientation.

        :param to_pose:
            The target pose
        :param max_distance:
            Maximum distance within which we'll count as being close, defaults to 0.001
        :param max_orientation_difference:
            Maximum number of radians we can be off the target pose's orientation to count as close, defaults to 1
            degree (calculated with ``radians(1)``)
        :return:
            True if this pose is regarded as close to the other, False otherwise
        """
        if self.distance_to_pose(to_pose) > max_distance:
            return False
        elif abs(smallest_difference(self.orientation, to_pose.orientation)) > max_orientation_difference:
            return False
        else:
            return True

    def translate(self, vector):
        """
        Create a new pose, with the same orientation as this one and the specified translation applied to its position.

        :param euclid.Vector2 vector:
            Vector by which the position of this pose should be translated when creating the new Pose
        :return:
            Returns the new Pose
        """
        return Pose(position=self.position + vector, orientation=self.orientation)

    def pose_to_pose_vector(self, to_pose):
        """
        Calculates the Vector2, in robot coordinate space (remember that Pose objects use world coordinates!) that
        represents the translation required to move from this Pose to the specified target Pose.

        :param approxeng.chassis.Pose to_pose:
            A target :class:`approxeng.chassis.Pose`, the resultant vector in robot space will translate the robot to
            the position contained in this pose. Note that this does not take any account of the orientation component
            of the to_pose, only the starting one.
        :return:
            A :class:`euclid.Vector2` containing the translation part, in robot space, of the motion required to move
            from this Pose to the target.
        """
        return rotate_vector(
            vector=Vector2(to_pose.position.x - self.position.x, to_pose.position.y - self.position.y),
            angle=-self.orientation)

    def pose_to_pose_motion(self, to_pose, time_seconds):
        """
        Calculates a Motion which should be applied to the current Pose to move the robot towards the target, such that
        it should hit the target at no less than time_seconds into the future. This function must be called on any Pose
        update, i.e. from a dead reckoning module, as it doesn't do any course planning (it would, for example, be
        possible to calculate a single constant motion to move in an arc to the target Pose, but this would be rather
        inefficient, better to incrementally home in on the target by repeatedly calling this function). To move as
        fast as possible to the target, set the time to something implausibly small, then use the chassis functions
        to limit the resultant motion to the range possible for the chassis. This would require some kind of motion
        limit to avoid skidding and messing up the Pose calculation logic.

        :param to_pose:
            A target :class:`approxeng.chassis.Pose`
        :param time_seconds:
            A the minimum number of seconds to transition to the target pose.
        :return:
            A :class:`approxeng.chassis.Motion` containing the motion required to attain the target pose in the
            specified time. This is highly likely to be impossible, in which case using the chassis functions to
            determine the wheel power and extract the scaling factor will give the actual time (ignoring acceleration
            limits) to transition to the target.
        """
        translation = self.pose_to_pose_vector(to_pose=to_pose)
        rotation = smallest_difference(self.orientation, to_pose.orientation)
        return Motion(translation=translation / time_seconds, rotation=rotation / time_seconds)

    def calculate_pose_change(self, motion, time_delta):
        """
        Given this as the starting Pose, a Motion and a time in seconds, calculate the resultant Pose at the end of the
        time interval.

        This makes use of the fact that if you travel in a consistent direction while turning at a constant rate you
        will describe an arc. By calculating the centre point of this arc we can simply rotate the starting pose around
        this centre point. This is considerably simpler than integrating over the motion 3-vector. A special case is
        used to avoid division by zero errors when there is no rotation component to the motion.

        :param approxeng.chassis.Motion motion:
            The motion of the robot, assumed to be constant for the duration of the time interval. The motion is
            expressed in the robot's coordinate frame, so a translation of (0,1) is always a forward motion,
            irrespective of the current orientation.
        :param float time_delta:
            The time in seconds during which the specified motion should be applied.
        :return:
            A :class:`approxeng.chassis.Pose` which represents resultant pose after applying the supplied motion for the
            given time.
        """

        # Total delta in orientation angle over the time interval
        orientation_delta = motion.rotation * time_delta
        # Scaled translation vector rotated into world coordinate space (motion uses robot space)
        translation_vector_world = rotate_vector(motion.translation, self.orientation) * time_delta
        ':type : euclid.Vector2'

        if orientation_delta == 0:
            # No orientation, trivially add the rotated, scaled, translation vector to the current pose
            return self.translate(translation_vector_world)
        else:
            centre_of_rotation = self.position + translation_vector_world.cross() / orientation_delta
            ':type : euclid.Point2'
            final_position = rotate_point(self.position, angle=orientation_delta, origin=centre_of_rotation)
            return Pose(position=final_position, orientation=self.orientation + orientation_delta)

    def __str__(self):
        return 'Pose[x={}, y={}, orientation={} (deg={})]'.format(self.position.x, self.position.y, self.orientation,
                                                                  degrees(self.orientation))


class Wheel:
    """
    A wheel is a mechanism to transfer rotational velocity into linear velocity. A conventional wheel, or an omni-wheel
    with rotors aligned at 90 degrees to the main wheel, impose their driving force perpendicular to the axis of the
    rotational axis. Other wheel types may translate the rotational velocity into linear movement vectors that are at
    arbitrary angles. We specify a wheel in terms of the linear translation that will be effected by a single rotation
    of the drive axle. In the case of omni or meccanum wheels this is the component of the motion along that vector, but
    there may be arbitrary amounts of motion perpendicular to this vector.

    Wheels are positioned at a particular location on the chassis.
    """

    __metaclass__ = ABCMeta

    def __init__(self, location, drive_vector, maximum_rotations_per_second, allows_slip=True):
        """
        :param euclid.Point2 location:
            The location of the contact point of the wheel relative to the robot chassis.
        :param euclid.Vector2 drive_vector:
            The drive vector corresponding to a single rotation of the wheel.
        :param maximum_rotations_per_second:
            Maximum rotations per second of the wheel.
        :param allows_slip:
            True if the wheel allows motion perpendicular to the drive vector, false if the only permitted motion of the
            wheel is along the drive.
        """
        self.location = location
        self.drive_vector = drive_vector
        self.maximum_rotation_per_second = maximum_rotations_per_second
        self.allows_slip = allows_slip
        self.vector_magnitude_squared = self.drive_vector.magnitude_squared()
        self.co_x = self.drive_vector.x / self.vector_magnitude_squared
        self.co_y = self.drive_vector.y / self.vector_magnitude_squared
        self.co_theta = (self.drive_vector.x * self.location.y -
                         self.drive_vector.y * self.location.x) / self.vector_magnitude_squared

    def speed(self, velocity):
        """
        Given a velocity at a wheel contact point, calculate the speed in revolutions per second at which the wheel
        should be driven.

        Method: we want to find the projection of the velocity onto the vector representing the drive of this wheel.
        We store the vector representing a single revolution of travel as self.drive_vector, so the projection onto this
        would be velocity.dot(self.drive_vector / abs(self.drive_vector)). However, we want revolutions per second, so
        we must then divide again by abs(self.drive_vector), leading to
        velocity.dot(self.vector / abs(self.vector))/abs(self.vector). Because the definition of the dot product is
        the sum of x1*x2, y1*y2, ... any scalar applied to each x, y ... of a single vector can be moved outside
        the dot product, so we can simplify as velocity.dot(self.vector) / abs(self.vector)^2. As the magnitude of
        the vector is taken by sqrt(x^2+y^2) we can simply express this as (x^2+y^2), held in the convenient
        function magnitude_squared(). So our final simplified form is
        velocity.dot(self.vector) / self.vector.magnitude_squared(). For efficiency, and because self.drive_vector
        doesn't change, we can pre-compute this.

        :param euclid.Vector2 velocity:
            The velocity at the wheel's contact point with the surface, expressed in mm/s
        :return:
            Target wheel speed in rotations per second to hit the desired vector at the contact point.
        """
        return velocity.dot(self.drive_vector) / self.vector_magnitude_squared


class OmniWheel(Wheel):
    """
    Defines a single omni-wheel within a chassis assembly. Omni-wheels are wheels formed from rollers, where the
    motion of the roller is perpendicular to the motion of the primary wheel. This is distinct from a mechanum wheel
    where the rollers are at an angle (normally around 40-30 degrees) to the primary wheel. Omni-wheels must be
    positioned on the chassis with non-parallel unit vectors, mechanum wheels can in some cases be positioned with
    all unit vectors parallel.

    A wheel has a location relative to the chassis centre and a vector describing the direction of motion of the
    wheel when driven with a positive angular velocity. The location is specified in millimetres, and the magnitude
    of the wheel vector should be equal to the number of millimetres travelled in a single revolution. This allows
    for different sized wheels to be handled within the same chassis.
    """

    def __init__(self, position, max_speed=0, angle=None, radius=None):
        """
        Create a new omni-wheel object, specifying the position and either a direction vector directly or the angle
        in degrees clockwise from the position Y axis along with the radius of the wheel.

        :param euclid.Point2 position:
            The wheel's contact point with the surface, specified relative to the centre of the
            chassis. Units are millimetres.
        :param float max_speed:
            The maximum number of revolutions per second allowed for this wheel. When calculating the wheel speeds
            required for a given trajectory this value is used to scale back all motion if any wheel would have to
            move at an impossible speed. If not specified this defaults to None, indicating that no speed limit
            should be placed on this wheel.
        :param angle:
            The angle, specified in radians from the positive Y axis where positive values are clockwise from this
            axis when viewed from above, of the direction of travel of the wheel when driven with a positive speed.
            If this value is specified then radius must also be specified and dx,dy left as None.
        :param radius:
            The radius in millimetres of the wheel, measuring from the centre to the contact point with the surface,
            this may be hard to determine for some wheels based on their geometry, particularly for wheels with
            cylindrical rollers, as the radius will vary. For these cases it may be worth directly measuring the
            circumference of the entire assembly and calculating radius rather than measuring directly. This is used
            to determine the magnitude of the direction vector. If this is not None then the angle must also be
            specified, and dx,dy left as None.
        """
        circumference = 2 * pi * radius
        self.vector = Vector2(sin(angle) * circumference, cos(angle) * circumference)
        super(OmniWheel, self).__init__(location=position,
                                        drive_vector=Vector2(sin(angle) * circumference, cos(angle) * circumference),
                                        maximum_rotations_per_second=max_speed, allows_slip=True)


class HoloChassis:
    """
    An assembly of wheels at various positions and angles, which can be driven independently to create a holonomic drive
    system. A holonomic system is one where number of degrees of freedom in the system is equal to the number of
    directly controllable degrees of freedom, so for a chassis intended to move in two dimensions the degrees of freedom
    are two axes of translation and one of rotation. For a full holonomic system we therefore need at least three wheels
    defined.
    """

    def __init__(self, wheels):
        """
        Create a new chassis, specifying a set of wheels.
        
        :param wheels:
            A sequence of :class:`approxeng.chassis.Wheel` objects defining the wheels for this chassis.
        """
        self.wheels = wheels
        if NUMPY_AVAILABLE:
            # If we have numpy available, calculate the matrix of coefficients for solving for motion from wheel speeds
            self._matrix_coefficients = np_array([[wheel.co_x, wheel.co_y, wheel.co_theta] for wheel in self.wheels])
        if len(wheels) == 3:
            w1 = wheels[0]
            w2 = wheels[1]
            w3 = wheels[2]
            # special case code to handle three wheeled chassis without requiring numpy
            self._green = w1.co_x * w2.co_y - w1.co_y * w2.co_x
            self._orange = w2.co_theta * w1.co_x - w1.co_theta * w2.co_x
            self._yellow = w3.co_theta * w1.co_x - w1.co_theta * w3.co_x
            # The large constant used as a divisor when calculating the first of the unknowns
            self._main_divisor = self._orange * (w3.co_y * w1.co_x - w1.co_y * w3.co_x) - self._yellow * self._green

    def calculate_motion(self, speeds):
        """
        Invert the motion to speed calculation to obtain the actual linear and angular velocity of the chassis given
        a vector of wheel speeds. 
        
        If there are exactly three speeds (and therefore the chassis has three wheels), an optimised version of the
        algorithm is used which solves explicitly, otherwise if numpy is available it's used to calculate a unique
        solution. The three-wheel explicit algorithm is approximately 5 times faster than numpy's linear solver
        
        See http://docs.scipy.org/doc/numpy-1.10.1/reference/generated/numpy.linalg.solve.html for more details on the
        linear solver

        If this function is called on a list of speeds of length other than 3 and numpy is not available an empty Motion
        will be returned

        :param speeds:
            An array of wheel speeds, expressed as floats with units of radians per second, positive being towards
            the wheel vector.
        :return:
            A :class:`approxeng.chassis.Motion` object containing the calculated translation and rotation in the robot's
            coordinate space.
        """
        if len(speeds) == 3:
            return self._calculate_motion_simple(speeds)
        elif NUMPY_AVAILABLE:
            return self._calculate_motion_numpy(speeds)
        else:
            return Motion(translation=Vector2(x=0, y=0), rotation=0)

    def _calculate_motion_numpy(self, speeds):
        """
        Invert the motion to speed calculation to obtain the actual linear and angular velocity of the chassis given
        a vector of wheel speeds. See http://docs.scipy.org/doc/numpy-1.10.1/reference/generated/numpy.linalg.solve.html

        Numpy must be available to use this function

        :param speeds:
            An array of wheel speeds, expressed as floats with units of radians per second, positive being towards
            the wheel vector.
        :return:
            A :class:`approxeng.chassis.Motion` object containing the calculated translation and rotation
            in the robot's coordinate space.
        """
        motion_array = np_solve(self._matrix_coefficients, np_array(speeds))
        return Motion(Vector2(x=float(motion_array[0]),
                              y=float(motion_array[1])),
                      rotation=float(motion_array[2]))

    def _calculate_motion_simple(self, speeds):
        """
        As _calculate_motion_numpy, but using an exact algebraic solution which doesn't require numpy. This variant 
        only works when there are exactly three wheels (in which case it's much faster), otherwise an empty Motion is 
        returned.
        
        :param speeds:
            An array of wheel speeds, expressed as floats with units of radians per second, positive being towards
            the wheel vector.
        :return:
            A :class:`approxeng.holochassis.chassis.Motion` object containing the calculated translation and rotation
            in the robot's coordinate space.
        """
        if len(speeds) != 3:
            return Motion()
        else:
            d1 = speeds[0]
            d2 = speeds[1]
            d3 = speeds[2]
            y = (self._orange * (self.wheels[0].co_x * d3 - self.wheels[2].co_x * d1) - self._yellow * (
                    self.wheels[0].co_x * d2 - self.wheels[1].co_x * d1)) / self._main_divisor
            theta = (self.wheels[0].co_x * d2 - self.wheels[1].co_x * d1 - self._green * y) / self._orange
            if self.wheels[0].co_x != 0:
                x = (d1 - self.wheels[0].co_y * y - self.wheels[0].co_theta * theta) / self.wheels[0].co_x
            elif self.wheels[1].co_x != 0:
                x = (d2 - self.wheels[1].co_y * y - self.wheels[1].co_theta * theta) / self.wheels[1].co_x
            else:
                x = (d3 - self.wheels[2].co_y * y - self.wheels[2].co_theta * theta) / self.wheels[2].co_x
            return Motion(x=x, y=y, rotation=theta)

    def get_max_translation_speed(self):
        """
        Calculate the maximum translation speed, assuming all directions are equivalent and that there is no rotation
        component to the motion.

        :return:
            Maximum speed in millimetres per second as a float
        """
        unrealistic_speed = 100000.0
        scaling = self.get_wheel_speeds(Motion(translation=Vector2(0, unrealistic_speed), rotation=0)).scaling
        return unrealistic_speed * scaling

    def get_max_rotation_speed(self):
        """
        Calculate the maximum rotation speed around the origin in radians per second, assuming no translation motion
        at the same time.

        :return:
            Maximum radians per second as a float
        """
        unrealistic_speed = 2 * pi * 100
        scaling = self.get_wheel_speeds(Motion(translation=Vector2(0, 0), rotation=unrealistic_speed)).scaling
        return unrealistic_speed * scaling

    def get_wheel_speeds(self, motion, origin=Point2(x=0, y=0)):
        """
        Calculate speeds to drive each wheel in the chassis at to attain the specified rotation / translation 3-vector.

        :param approxeng.chassis.Motion motion:
            Desired motion of the robot chassis
        :param euclid.Point2 origin:
            Optional, can define the centre of rotation to be something other than 0,0. Units are in millimetres.
            Defaults to rotating around x=0, y=0.
        :return:
            A :class:`approxeng.chassis.WheelSpeeds` containing both the target wheel speeds and the scaling, if
            any, which was required to bring those speeds into the allowed range for all wheels. This prevents
            unexpected motion in cases where only a single wheel is being asked to turn too fast, in such cases all
            wheel speeds will be scaled back such that the highest is within the bounds allowed for that particular
            wheel. This can accommodate wheels with different top speeds.
        """

        def velocity_at(point):
            """
            Compute the velocity as a Vector2 at the specified point given the enclosing translation and rotation values

            Method: Normalise the vector from the origin to the point, then take the cross of itself to produce a unit
            vector with direction that of a rotation around the origin. Scale this by the distance from the origin and
            by the rotation in radians per second, then simply add the translation vector.

            :param euclid.Point2 point:
                Point at which to calculate velocity
            :return:
                A :class:`euclid.Vector2` representing the velocity at the specified point in mm/s
            """
            d = point - origin
            return d.cross() * motion.rotation + motion.translation

        wheel_speeds = list(wheel.speed(velocity_at(wheel.location)) for wheel in self.wheels)
        scale = 1.0
        for speed, wheel in zip(wheel_speeds, self.wheels):
            if wheel.maximum_rotation_per_second is not None and abs(speed) > wheel.maximum_rotation_per_second:
                wheel_scale = wheel.maximum_rotation_per_second / abs(speed)
                scale = min(scale, wheel_scale)
        return WheelSpeeds(speeds=list(speed * scale for speed in wheel_speeds), scaling=scale)
