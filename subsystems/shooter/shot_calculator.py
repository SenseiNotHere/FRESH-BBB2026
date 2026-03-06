import math
from typing import List

from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Pose2d, Pose3d, Translation2d, Rotation2d

from constants.field_constants import Hub
from constants.constants import ShooterConstants


class ShotCalculator(Subsystem):
    """
    Computes distance-based shooter speed and yaw.
    Does NOT control shooter. Pure calculation subsystem.

    Credits to FRC Team 868 - TechHOUNDS
    """

    def __init__(self, drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self._target_speed_rps = 0.0

    # Periodic

    def periodic(self):

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            target_location = Hub.RED_HUB
        else:
            target_location = Hub.BLUE_HUB

        drivetrain_pose: Pose2d = self.drivetrain.getPose()
        drivetrain_location = drivetrain_pose.translation()

        # 2D Distance
        _target_distance = drivetrain_location.distance(target_location)

        # Distance -> Speed Lookup
        lookup = ShooterConstants.DISTANCE_TO_RPS
        self._target_speed_rps = lookup.get(_target_distance)

        # Effective target (future SOTM logic goes here)
        _effective_target_pose = target_location - drivetrain_location

        vector_to_goal = target_location - drivetrain_location

        _effective_yaw = vector_to_goal.angle().radians()
        SmartDashboard.putNumber("ShotCalc/EffectiveYaw", 180 * _effective_yaw / math.pi)
        SmartDashboard.putNumber("ShotCalc/Distance", _target_distance)

        if self.drivetrain.field is not None:
            self.drivetrain.field.getObject("shot-calc-dir").setPoses(draw_arrow(drivetrain_location, vector_to_goal))

    # Public API

    def getTargetSpeedRPS(self) -> float:
        return self._target_speed_rps


def draw_arrow(start: Translation2d, directionVector: Translation2d, nPoints=11, size=0.85, tip=0.1) -> List[Pose2d]:
    result = []
    length = directionVector.norm()
    zero = Rotation2d(0)
    if length > 0:
        end = start
        directionVector = directionVector / length
        for i in range(nPoints):
            end = start + directionVector * (size * i / nPoints)
            result.append(Pose2d(end, zero))
        ray1 = directionVector.rotateBy(Rotation2d.fromDegrees(90)) * tip
        ray2 = directionVector * tip
        ray3 = directionVector.rotateBy(Rotation2d.fromDegrees(-90)) * tip
        result.append(Pose2d(end + ray1, zero))
        result.append(Pose2d(end + ray2, zero))
        result.append(Pose2d(end + ray3, zero))
        result.append(Pose2d(end, zero))
    return result
