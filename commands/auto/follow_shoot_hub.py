from commands2 import ParallelCommandGroup
from commands.drive.point_torwards_location import PointTowardsLocation
from superstructure.robot_state import RobotState
from superstructure.superstructure import Superstructure
from subsystems.drive.drivesubsystem import DriveSubsystem
from constants.field_constants import Hub


class FollowShootHub(ParallelCommandGroup):

    def __init__(
        self,
        superstructure: Superstructure,
        drivetrain: DriveSubsystem,
    ):

        point_cmd = PointTowardsLocation(
            drivetrain=drivetrain,
            location=Hub.BLUE_HUB,
            locationIfRed=Hub.RED_HUB
        )

        state_cmd = superstructure.createStateCommand(RobotState.PREP_SHOT)

        super().__init__(point_cmd, state_cmd)

        self.superstructure = superstructure
        self.drivetrain = drivetrain

    def end(self, interrupted: bool):
        self.superstructure.setState(RobotState.IDLE)
        self.drivetrain.stop()