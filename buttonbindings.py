from commands2 import InstantCommand
from wpilib import XboxController

from commands import ResetXY, ResetSwerveFront, FollowShootHub, ToggleIntakePositionCommand
from superstructure import RobotState


class ButtonBindings:
    def __init__(self, robot_container):
        """Initialize ButtonBindings with access to the robot container."""
        self.robotContainer = robot_container

        # Core subsystems
        self.drivetrain = robot_container.vroomvroom
        self.superstructure = robot_container.megamente
        self.driverController = robot_container.vroomvroomController
        self.operatorController = robot_container.statesideController
        self.limelight = robot_container.lemon
        self.orchestra = robot_container.orca

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
                drivetrain=self.drivetrain,
                reason="pov(0)"
            )
        )

        # Reset Robot Front
        self.driverController.pov(180).onTrue(
            ResetSwerveFront(self.drivetrain)
        )
        
        # X-Break
        self.driverController.pov(270).whileTrue(
            InstantCommand(self.drivetrain.setX, self.drivetrain)
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
#        self.driverController.button(
#            XboxController.Button.kRightBumper
#        ).whileTrue(
#            FollowShootHub(self.superstructure, self.drivetrain)
#        )

    # Operator Controls

    def _configureOperatorBindings(self):

        # Right Trigger = Intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kRightTrigger,
            threshold=0.05
        ).whileTrue(
            self.superstructure.createStateCommand(RobotState.INTAKING)
        )

        # Left Trigger = Deploy / Stow Intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kLeftTrigger,
            threshold=0.05
        ).whileTrue(
            ToggleIntakePositionCommand(self.superstructure)
        )
        
        # A Button = Play Song
        self.operatorController.button(
            XboxController.Button.kA
        ).whileTrue(
            self.superstructure.createStateCommand(RobotState.PLAYING_SONG)
        )