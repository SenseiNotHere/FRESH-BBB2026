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

        # Computed outputs
        self._target_distance: float = 0.0
        self._target_speed_rps: float = 0.0
        self._effective_target_pose: Pose3d = Hub.CENTER
        self._effective_yaw: float = 0.0

    # Periodic

    def periodic(self):

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            target_location = Hub.RED_HUB
        else:
            target_location = Hub.BLUE_HUB

        drivetrain_pose: Pose2d = self.drivetrain.getPose()

        # 2D Distance
        self._target_distance = (
            drivetrain_pose.translation()
            .distance(target_location)
        )

        # Distance -> Speed Lookup
        lookup = ShooterConstants.DISTANCE_TO_RPS
        self._target_speed_rps = lookup.get(self._target_distance)

        # Effective target (future SOTM logic goes here)
        self._effective_target_pose = target_location

        relative_pose = (
            target_location
        )

        target_location =

        self._effective_yaw = relative_pose.rotation().radians()
        SmartDashboard.putNumber("ShotCalc/EffectiveYaw", 180 * self._effective_yaw / math.pi)
        SmartDashboard.putNumber("ShotCalc/Distance", self._target_distance)

        if self.drivetrain.field is not None:
            vector_to_goal = target_location.translation().toTranslation2d() - drivetrain_pose.translation()
            self.drivetrain.field.getObject("shot-calc-dir").setPoses(draw_arrow(drivetrain_pose.translation(), vector_to_goal))

    # Public API

    def getTargetDistance(self) -> float:
        return self._target_distance

    def getTargetSpeedRPS(self) -> float:
        return self._target_speed_rps

    def getEffectiveTargetPose(self) -> Pose3d:
        return self._effective_target_pose

    def getEffectiveYaw(self) -> float:
        return self._effective_yaw


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
