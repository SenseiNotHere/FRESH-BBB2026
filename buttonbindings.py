from wpilib import XboxController, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from commands2 import InstantCommand

from commands.auto.follow_shoot_hub import FollowShootHub
from commands.auto.pose_lock_dock import PoseLockDock
from commands.climber.climber_commands import ToggleClimbAuto
from commands.intake.intake_commands import RunIntake, ReverseIntake, DeployRetractIntake
from superstructure.robot_state import RobotState

from constants.field_constants import AprilTags

from commands.drive.reset_xy import ResetXY, ResetSwerveFront
from commands.auto.drive_torwards_object import SwerveTowardsObject

from utils.fieldutils import getClosestClimbPose


class ButtonBindings:
    def __init__(self, robot_container):
        """Initialize ButtonBindings with access to the robot container."""
        self.robotContainer = robot_container

        # Core subsystems
        self.robotDrive = robot_container.robotDrive
        self.superstructure = robot_container.superstructure
        self.driverController = robot_container.driverController
        self.operatorController = robot_container.operatorController
        self.limelight = robot_container.limelight
        self.orchestra = robot_container.orchestra

    # Main Binding Configuration

    def configureButtonBindings(self):
        self._configureDriverBindings()
        self._configureOperatorBindings()

    # Driver Controls

    def _configureDriverBindings(self):
        # Reset Controls
        self.driverController.pov(0).onTrue(
            ResetXY(
                x=0.0,
                y=0.0,
                headingDegrees=0.0,
                drivetrain=self.robotDrive
            )
        )

        self.driverController.pov(180).onTrue(
            ResetSwerveFront(self.robotDrive)
        )

        self.driverController.pov(270).whileTrue(
            InstantCommand(self.robotDrive.setX, self.robotDrive)
        )

        # Shooter
        # Right Bumper = Shoot
        self.driverController.button(
            XboxController.Button.kRightBumper
        ).whileTrue(
            self.superstructure.createStateCommand(
                RobotState.PREP_SHOT
            )
        )

        # Right Trigger = Follow hub and shoot
        self.driverController.axisGreaterThan(
            XboxController.Axis.kRightTrigger,
            threshold=0.05
        ).whileTrue(
            FollowShootHub(self.robotContainer.superstructure, self.robotDrive)
        )

        # Left Trigger = Tower dock
        targetPos = getClosestClimbPose(self.robotDrive)

#        self.driverController.axisGreaterThan(
#            XboxController.Axis.kLeftTrigger,
#            threshold=0.05
#        ).whileTrue(
#            PoseLockDock(self.robotDrive, targetPos)
#        )

        # X-Button = MUSICCC
        self.driverController.button(
            XboxController.Button.kX
        ).whileTrue(
            self.superstructure.createStateCommand(RobotState.PLAYING_SONG)
        )

    # Operator Controls

    def _configureOperatorBindings(self):
        # Drawer / Elevator Controls
        # Elevator Down
        self.operatorController.button(
            XboxController.Button.kB
        ).whileTrue(
            self.superstructure.createStateCommand(
                RobotState.ELEVATOR_LOWERING
            )
        )

        # Elevator Up
        self.operatorController.button(
            XboxController.Button.kY
        ).whileTrue(
            self.superstructure.createStateCommand(
                RobotState.ELEVATOR_RISING
            )
        )

        # Elevator Minimum
        self.operatorController.button(
            XboxController.Button.kX
        ).whileTrue(
            self.superstructure.createStateCommand(
                RobotState.ELEVATOR_MINIMUM
            )
        )

        # Intake Controls
        # A = Deploy/Stow Intake
        self.operatorController.button(
            XboxController.Button.kA
        ).onTrue(
            DeployRetractIntake(self.robotContainer.superstructure)
        )

        # Right Trigger = Intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kRightTrigger,
            threshold=0.05
        ).whileTrue(
            RunIntake(self.robotContainer.superstructure)
        )

        # Left Trigger = Reverse intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kLeftTrigger,
            threshold=0.05
        ).whileTrue(
            ReverseIntake(self.robotContainer.superstructure)
        )

    # Helpers

    def _log_and_get_april_tag_position(self, tag_id_callable, tag_id_name):
        tag_id = tag_id_callable()
        SmartDashboard.putString(
            f"command/c{self.__class__.__name__}/{tag_id_name}",
            f"Tag ID: {tag_id}"
        )
        position = AprilTags.APRIL_TAG_POSITIONS.get(tag_id)
        SmartDashboard.putString(
            f"command/c{self.__class__.__name__}/{tag_id_name}_position",
            f"Position: {position}"
        )
        return position
