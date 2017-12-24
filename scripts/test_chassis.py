from time import time

from approxeng.chassis import HoloChassis, Motion, OmniWheel
from approxeng.chassis.util import get_regular_triangular_chassis
from euclid import Point2, Vector2


def test_timing(iterations=1000):
    chassis = get_regular_triangular_chassis(200, 20, 20)
    start = time()
    for _ in range(iterations):
        chassis.calculate_motion([20, 20, 20])
    end = time()
    print('numpy based, {} for {} iterations'.format(end - start, iterations))
    start = time()
    for _ in range(iterations):
        chassis._calculate_motion_simple([20, 20, 20])
    end = time()
    print('simple, {} for {} iterations'.format(end - start, iterations))
    print('Using numpy: {}'.format(str(chassis._calculate_motion_numpy([20, 20, 20]))))
    print('Using simple: {}'.format(str(chassis._calculate_motion_simple([20, 20, 20]))))


def test():
    chassis = HoloChassis(wheels=[
        OmniWheel(position=Point2(1, 0), angle=0, radius=60),
        OmniWheel(position=Point2(-1, 0), angle=0, radius=60)]
    )
    print(chassis.get_wheel_speeds(Motion(translation=Vector2(0, 0), rotation=0.5)))
    print(chassis.get_wheel_speeds(Motion(translation=Vector2(0, 0), rotation=0.5), origin=Point2(1, 0)))


test_timing()
test()
