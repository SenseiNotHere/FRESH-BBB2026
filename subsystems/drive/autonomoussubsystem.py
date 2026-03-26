from typing import TYPE_CHECKING
from xml.dom.minidom import NamedNodeMap

from commands2 import Subsystem, ParallelCommandGroup, WaitCommand
from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.auto import NamedCommands, EventTrigger

from wpimath.kinematics import ChassisSpeeds

from commands.drive.point_torwards_location import PointTowardsLocation, PointTowardsLocationAuto
from constants import AutoConstants, Hub

from .drivesubsystem import DriveSubsystem
from superstructure import Superstructure, RobotState, RobotReadiness

from utils import log

from commands.intake.intake_position import DeployIntake, StowIntake, RunIntakeRollers, DoIntake

if TYPE_CHECKING:
    from robotcontainer import RobotContainer


class AutonomousSubsystem(Subsystem):
    def __init__(
            self,
            drivetrain: DriveSubsystem | None = None,
            robotContainer: "RobotContainer" = None,
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

        point_cmd = PointTowardsLocation(
            drivetrain=self.robotContainer.vroomvroom,
            location=Hub.BLUE_HUB,
            locationIfRed=Hub.RED_HUB
        )
        shootCmd8s = self.superstructure.createStateCommand(RobotState.PREP_SHOT).alongWith(
            PointTowardsLocationAuto(
                drivetrain=self.robotContainer.vroomvroom,
                location=Hub.BLUE_HUB,
                locationIfRed=Hub.RED_HUB
            )
        ).withTimeout(8.0)

        NamedCommands.registerCommand('PREP_SHOT_5s', self.superstructure.createStateCommand(RobotState.PREP_SHOT).withTimeout(5.0))
        NamedCommands.registerCommand('PREP_SHOT_8s', shootCmd8s)
        NamedCommands.registerCommand('INTAKING', RunIntakeRollers(self.robotContainer.gulp).withTimeout(2.0))
        NamedCommands.registerCommand('INTAKE_DEPLOYED', DeployIntake(self.robotContainer.gulp))
        NamedCommands.registerCommand('INTAKE_STOWED', StowIntake(self.robotContainer.gulp))
        NamedCommands.registerCommand('DO_INTAKE_4s', DoIntake(self.robotContainer.gulp).withTimeout(8.0))
        NamedCommands.registerCommand('POINT_HUB', WaitCommand(0))

    def registerEventTriggers(self):
        pass

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

            self.drivetrain.field.getObject("Auto Path").setPoses(poses)

        except Exception as e:
            log("Autonomous",f"Failed to draw auto '{autoName}': {e}")

    def clearAutoPreview(self):
        self.drivetrain.field.getObject("Auto Path").setPoses([])