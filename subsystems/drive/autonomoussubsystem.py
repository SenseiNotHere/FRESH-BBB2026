from typing import TYPE_CHECKING
from xml.dom.minidom import NamedNodeMap

from commands2 import Subsystem, ParallelCommandGroup, WaitCommand, SequentialCommandGroup, FunctionalCommand
from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.auto import NamedCommands, EventTrigger
from pathplannerlib.util import FlippingUtil

from wpimath.kinematics import ChassisSpeeds

from constants import AutoConstants, Hub

from .drivesubsystem import DriveSubsystem
from utils import log

from commands.intake.intake_position import DeployIntake, StowIntake, RunIntakeRollers, DoIntake, PulseIntake

if TYPE_CHECKING:
    from robotcontainer import RobotContainer
    from superstructure import Superstructure


class AutonomousSubsystem(Subsystem):
    def __init__(
            self,
            drivetrain: DriveSubsystem,
            robotContainer: "RobotContainer"
    ):
        """
        Autonomous Subsystem class. Handles all autonomous-related functionality.
        This is a singleton class. Meaning there should only ever be one instance of this class.

        :param drivetrain: The drivetrain subsystem.
        :param robotContainer: The robot container.
        """
        super().__init__()

        self.drivetrain = drivetrain
        self.robotContainer = robotContainer
        self.superstructure: Superstructure = robotContainer.megamente

        # Register commands and event triggers
        self.registerNamedCommands()
        self.registerEventTriggers()

        AutoBuilder.configure(
            self._getPose,
            self._resetOdometry,
            self._getRobotRelativeSpeeds,
            self._driveRobotRelative,
            PPHolonomicDriveController(
                PIDConstants(AutoConstants.kPController, 0, 0),
                PIDConstants(AutoConstants.kPThetaController, 0, 0),
            ),
            AutoConstants.config,
            self.shouldFlipPath,
            self.drivetrain
        )

    def registerNamedCommands(self):
        from commands.drive.point_torwards_location import PointTowardsLocationAuto
        from superstructure import RobotState

        point_cmd2s = PointTowardsLocationAuto(
            drivetrain=self.robotContainer.vroomvroom,
            location=Hub.BLUE_HUB,
            locationIfRed=Hub.RED_HUB
        ).withTimeout(1.0)
        shootCmd6s = self.superstructure.createStateCommand(RobotState.PREP_SHOT).alongWith(
            PointTowardsLocationAuto(
                drivetrain=self.robotContainer.vroomvroom,
                location=Hub.BLUE_HUB,
                locationIfRed=Hub.RED_HUB
            ), PulseIntake(self.robotContainer.gulp, True)
        ).withTimeout(6.0)
        pointAndShoot = SequentialCommandGroup(point_cmd2s, shootCmd6s)
        # General
        NamedCommands.registerCommand('DEPLOY_INTAKE', DeployIntake(self.robotContainer.gulp))
        NamedCommands.registerCommand('POINT_AND_SHOOT', pointAndShoot)

        # Driver Station 2
        pointToHub = PointTowardsLocationAuto(
            drivetrain=self.robotContainer.vroomvroom,
            location=Hub.BLUE_HUB,
            locationIfRed=Hub.RED_HUB
        ).withTimeout(1.0)
        shootCmdDS2 = self.superstructure.createStateCommand(RobotState.PREP_SHOT).withTimeout(8.0)
        pulseCmd = PulseIntake(self.robotContainer.gulp, deploy_at_end=True)
        NamedCommands.registerCommand('PREP_SHOT_DS2', pointToHub.andThen(shootCmdDS2.alongWith(pulseCmd)))

    def registerEventTriggers(self):
        EventTrigger('DEPLOY_INTAKE').onTrue(DeployIntake(self.robotContainer.gulp))
        EventTrigger('INTAKING').whileTrue(RunIntakeRollers(self.robotContainer.gulp))
        EventTrigger('INTAKING_DS2_NEUTRAL').whileTrue(RunIntakeRollers(self.robotContainer.gulp))

    def _driveRobotRelative(self, speeds, feedforwards):
        self.drivetrain.driveRobotRelativeChassisSpeeds(
            ChassisSpeeds(speeds.vx, speeds.vy, -speeds.omega),
            feedforwards
        )

    def _getRobotRelativeSpeeds(self):
        return self.drivetrain.getRobotRelativeSpeeds()

    def _resetOdometry(self, pose):
        self.drivetrain.resetOdometry(pose)

    def _getPose(self):
        return self.drivetrain.getPose()

    def shouldFlipPath(self) -> bool:
        alliance = self.drivetrain.getAlliance()
        return alliance == DriverStation.Alliance.kRed


    def drawAuto(self, autoName: str):
        if not autoName:
            return

        try:
            paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName)

            poses = [pose for path in paths for pose in path.getPathPoses()]

            if self.shouldFlipPath():
                poses = [FlippingUtil.flipFieldPose(pose) for pose in poses]

            self.drivetrain.field.getObject("Auto Path").setPoses(poses)

        except Exception as e:
            log("Autonomous", f"Failed to draw auto '{autoName}': {e}")

    def clearAutoPreview(self):
        self.drivetrain.field.getObject("Auto Path").setPoses([])