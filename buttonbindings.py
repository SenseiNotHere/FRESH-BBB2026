from commands2 import InstantCommand
from wpilib import XboxController

from commands import ResetXY, ResetSwerveFront, FollowShootHub
from superstructure import RobotState


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
        # Reset XYZ
        self.driverController.pov(0).onTrue(
            ResetXY(
                x=0.0,
                y=0.0,
                headingDegrees=0.0,
                drivetrain=self.robotDrive,
                reason="pov(0)"
            )
        )

        # Reset Robot Front
        self.driverController.pov(180).onTrue(
            ResetSwerveFront(self.robotDrive)
        )
        
        # X-Break
        self.driverController.pov(270).whileTrue(
            InstantCommand(self.robotDrive.setX, self.robotDrive)
        )

        # Shooter
        # Right Trigger = Prep Shot
        self.driverController.axisGreaterThan(
            XboxController.Axis.kRightTrigger,
            0.1
        ).whileTrue(
            self.superstructure.createStateCommand(RobotState.PREP_SHOT)
        )

        # Follow Shoot Hub
        # Right Bumper = Follow Shoot Hub
        self.driverController.button(
            XboxController.Button.kRightBumper
        ).whileTrue(
            FollowShootHub(self.superstructure, self.robotDrive)
        )

    # Operator Controls

    def _configureOperatorBindings(self):

        # Right Trigger = Intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kRightTrigger,
            threshold=0.05
        ).whileTrue(
            self.superstructure.createStateCommand(RobotState.INTAKING)
        )