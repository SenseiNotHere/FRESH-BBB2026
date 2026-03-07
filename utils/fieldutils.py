from wpilib import DriverStation
from wpimath.geometry import Pose2d
import math

from constants.field_constants import Tower


def getClosestClimbPose(drivetrain) -> Pose2d:
    pose = drivetrain.getPose()

    isRed = drivetrain.getAlliance() == DriverStation.Alliance.kRed

    if isRed:
        top = Tower.RED_TOWER_CLIMB_TOP
        bottom = Tower.RED_TOWER_CLIMB_BOTTOM
    else:
        top = Tower.BLUE_TOWER_CLIMB_TOP
        bottom = Tower.BLUE_TOWER_CLIMB_BOTTOM

    # Compare full 2D distance
    distTop = math.hypot(pose.X() - top.X(), pose.Y() - top.Y())
    distBottom = math.hypot(pose.X() - bottom.X(), pose.Y() - bottom.Y())

    return top if distTop < distBottom else bottom