from typing import TYPE_CHECKING

from commands2 import Subsystem
from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.auto import NamedCommands, EventTrigger

from wpimath.kinematics import ChassisSpeeds

from constants.constants import AutoConstants

from .drivesubsystem import DriveSubsystem
from superstructure import Superstructure, RobotState, RobotReadiness

from utils import log

if TYPE_CHECKING:
    from robotcontainer import RobotContainer


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
        NamedCommands.registerCommand('PREP_SHOT', self.robotContainer.superstructure.createStateCommand(RobotState.PREP_SHOT))
        NamedCommands.registerCommand('INTAKING', self.robotContainer.superstructure.createStateCommand(RobotState.INTAKING))
        NamedCommands.registerCommand('INTAKE_DEPLOYED', self.robotContainer.superstructure.createStateCommand(RobotState.INTAKE_DEPLOYED))
        NamedCommands.registerCommand('INTAKE_STOWED', self.robotContainer.superstructure.createStateCommand(RobotState.INTAKE_STOWED))

    def registerEventTriggers(self):
        EventTrigger('INTAKE_DEPLOYED').whileTrue(self.robotContainer.superstructure.createStateCommand(RobotState.INTAKE_DEPLOYED))
        EventTrigger('INTAKE_STOWED').whileTrue(self.robotContainer.superstructure.createStateCommand(RobotState.INTAKE_STOWED))

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

    def shouldFlipPath(self):
        return self.drivetrain.getAlliance() == DriverStation.Alliance.kRed

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