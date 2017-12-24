from math import cos, radians, pi

from approxeng.chassis import Wheel, HoloChassis, rotate_point, rotate_vector
from euclid import Point2, Vector2


def get_regular_triangular_chassis(wheel_distance, wheel_radius, max_rotations_per_second):
    """
    Build a HoloChassis object with three wheels, each identical in size and maximum speed. Each wheel is positioned
    at the corner of a regular triangle, and with direction perpendicular to the normal vector at that corner.

    :param wheel_distance:
        Distance in millimetres between the contact points of each pair of wheels (i.e. the length of each edge of the
        regular triangle)
    :param wheel_radius:
        Wheel radius in millimetres
    :param max_rotations_per_second:
        Maximum wheel speed in revolutions per second
    :return:
        An appropriately configured HoloChassis
    """
    point = Point2(0, cos(radians(30)) * wheel_distance / 2.0)
    vector = Vector2(-2 * pi * wheel_radius, 0)

    # Pink
    wheel_a = Wheel(
        location=point,
        drive_vector=vector,
        maximum_rotations_per_second=max_rotations_per_second, allows_slip=True)
    # Yellow
    wheel_b = Wheel(
        location=rotate_point(point, pi * 2 / 3),
        drive_vector=rotate_vector(vector, pi * 2 / 3),
        maximum_rotations_per_second=max_rotations_per_second, allows_slip=True)
    # Green
    wheel_c = Wheel(
        location=rotate_point(point, pi * 4 / 3),
        drive_vector=rotate_vector(vector, pi * 4 / 3),
        maximum_rotations_per_second=max_rotations_per_second, allows_slip=True)

    return HoloChassis(wheels=[wheel_a, wheel_b, wheel_c])
