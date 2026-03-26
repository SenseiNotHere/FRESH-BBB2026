from commands2 import InstantCommand, ParallelCommandGroup
from wpilib import XboxController

from commands import ResetXY, ResetSwerveFront, FollowShootHub
from commands.drive.point_torwards_location import PointTowardsLocation
from commands.intake.intake_position import StowIntake, DeployIntake, PulseIntake, RunIntakeRollers, \
    RunIntakeRollersInverse, DoIntake
from superstructure import RobotState

from constants import Hub


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
        shootCmd = self.superstructure.createStateCommand(RobotState.PREP_SHOT)
        pulseCmd = PulseIntake(self.robotContainer.gulp, deploy_at_end=True)
        shootAndPulse = shootCmd.deadlineWith(pulseCmd)
    
        self.driverController.axisGreaterThan(
            XboxController.Axis.kRightTrigger,
            0.1
        ).whileTrue(
            shootAndPulse
        )

        # Follow Shoot Hub
        # Right Bumper = Follow Shoot Hub
        point_cmd = PointTowardsLocation(
            drivetrain=self.robotContainer.vroomvroom,
            location=Hub.BLUE_HUB,
            locationIfRed=Hub.RED_HUB
        )
        pointAndShoot = ParallelCommandGroup(point_cmd)

        self.driverController.button(
            XboxController.Button.kRightBumper
        ).whileTrue(
            pointAndShoot
        )

    # Operator Controls

    def _configureOperatorBindings(self):

        # Right Trigger = Intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kRightTrigger,
            threshold=0.05
        ).whileTrue(
            DoIntake(self.robotContainer.gulp)
        )
        
        # X Button = Run Intake (RunIntakeRollers)
        self.operatorController.button(
            XboxController.Button.kX
        ).whileTrue(
            RunIntakeRollers(self.robotContainer.gulp)
        )
        
        # Left Trigger = Reverse Intake
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kLeftTrigger,
            threshold=0.05
        ).whileTrue(
            RunIntakeRollersInverse(self.robotContainer.gulp)
        )

        # Left Bumper = Stow
        self.operatorController.button(
             XboxController.Button.kLeftBumper
        ).onTrue(
            StowIntake(self.robotContainer.gulp)
            # StowIntake(self.robotContainer.gulp)
        )

        # Right Bumper = Deploy
        self.operatorController.button(
            XboxController.Button.kRightBumper
        ).onTrue(
            DeployIntake(self.robotContainer.gulp)
        )

        # A Button = Play Song
        self.operatorController.button(
            XboxController.Button.kA
        ).whileTrue(
            InstantCommand(lambda: self.robotContainer.gulp.driveDeployMotor(0.5))
        )
        
        self.operatorController.button(
            XboxController.Button.kB
        ).whileTrue(
            InstantCommand(lambda: self.robotContainer.gulp.driveDeployMotor(-0.5))
        )