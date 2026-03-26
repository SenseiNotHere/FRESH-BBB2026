from typing import TYPE_CHECKING
from xml.dom.minidom import NamedNodeMap

from commands2 import Subsystem, ParallelCommandGroup
from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.auto import NamedCommands, EventTrigger

from wpimath.kinematics import ChassisSpeeds

from commands.drive.point_torwards_location import PointTowardsLocation
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

        point_cmd = PointTowardsLocation(
            drivetrain=self.robotContainer.vroomvroom,
            location=Hub.BLUE_HUB,
            locationIfRed=Hub.RED_HUB
        )
        
        shootCmd = self.superstructure.createStateCommand(RobotState.PREP_SHOT)
        
        pointAndShoot = ParallelCommandGroup(point_cmd, shootCmd)

        NamedCommands.registerCommand('PREP_SHOT_COMMAND', self.superstructure.createStateCommand(RobotState.PREP_SHOT))
        NamedCommands.registerCommand('POINT_AND_SHOOT', pointAndShoot)
#        NamedCommands.registerCommand('INTAKING', RunIntakeRollers(self.robotContainer.gulp).withTimeout(2.0))
#        NamedCommands.registerCommand('INTAKE_DEPLOYED', DeployIntake(self.robotContainer.gulp))
#        NamedCommands.registerCommand('INTAKE_STOWED', StowIntake(self.robotContainer.gulp))

    def registerEventTriggers(self):
        EventTrigger('INTAKE_DEPLOYED').onTrue(DeployIntake(self.robotContainer.gulp))
        EventTrigger('INTAKE_STOWED').onTrue(StowIntake(self.robotContainer.gulp))
        EventTrigger('INTAKING').whileTrue(RunIntakeRollers(self.robotContainer.gulp))
        EventTrigger('DO_INTAKE').whileTrue(DoIntake(self.robotContainer.gulp))
        EventTrigger('PREP_SHOT').onTrue(self.superstructure.createStateCommand(RobotState.PREP_SHOT))

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