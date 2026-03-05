from __future__ import annotations

import typing
import commands2

from commands2 import InstantCommand
from commands2.button import CommandGenericHID

from pathplannerlib.auto import NamedCommands, EventTrigger

from wpilib import (
    XboxController,
    SmartDashboard,
    SendableChooser,
    DriverStation
)

from wpimath.geometry import (
    Rotation2d,
    Translation3d,
)

from subsystems.drive.drivesubsystem import DriveSubsystem, BadSimPhysics, AutoBuilder
from subsystems.shooter.agitadorsubsystem import Agitator
from subsystems.vision.limelightcamera import LimelightCamera
from subsystems.vision.limelight_localizer import LimelightLocalizer
from subsystems.shooter.shootersubsystem import Shooter
from subsystems.shooter.indexersubsystem import Indexer
from subsystems.climber.climbersubsystem import Climber
from subsystems.intake.intakesubsystem import Intake
from subsystems.pneumatics.pneumaticssubsystem import Pneumatics
from subsystems.shooter.shot_calculator import ShotCalculator
from subsystems.orchestra.orchestrasubsystem import OrchestraSubsystem
from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState
from commands.drive.holonomic_drive import HolonomicDrive

from commands.climber.climber_commands import ManualClimb

from buttonbindings import ButtonBindings

from constants.constants import (
    OIConstants,
    ShooterConstants,
    IndexerConstants,
    ClimberConstants,
    IntakeConstants,
    PneumaticsConstants, AgitatorConstants
)


class RobotContainer:
    """
    The container for the robot. Subsystems are initialized here,
    button bindings are set up, and auto chooser is sent to the dashboard.
    """

    def __init__(self, robot):

        # Drive Subsystem

        self.robotDrive = DriveSubsystem()

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)

        # Test Chooser
        self.testChooser = SendableChooser()
        SmartDashboard.putData("Test Chooser", self.testChooser)

        # Controllers
        self.driverController = CommandGenericHID(
            OIConstants.kDriverControllerPort
        )
        self.operatorController = CommandGenericHID(
            OIConstants.kOperatorControllerPort
        )

        # Vision / Localization
        self.localizer = LimelightLocalizer(
            drivetrain=self.robotDrive,
            flipIfRed=False,
        )

        self.limelight = LimelightCamera("limelight-front")
        self.limelight.setPiPMode(1)
        self.limelightBack = LimelightCamera("limelight-back")

        self.localizer.addCamera(
            camera=self.limelight,
            cameraPoseOnRobot=Translation3d(0.0, 0.0, 0.0),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(0),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )

#        self.localizer.addCamera(
#            camera=self.limelightBack,
#            cameraPoseOnRobot=Translation3d(0.0, 0.0, 0.0),
#            cameraHeadingOnRobot=Rotation2d.fromDegrees(180),
#            minPercentFrame=0.07,
#             maxRotationSpeed=720,
#        )

        # Default Drive Command
        redAlliance = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kRightX),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=False,
                square=True,
            )
        )

        # Pneumatics

        self.pneumatics = Pneumatics(
            moduleID=PneumaticsConstants.kPCMID,
            moduleType=PneumaticsConstants.kModuleType
        )

        # Subsystems

        # Shooter
        self.shooter = Shooter(
            motorCANID=ShooterConstants.kShooterMotorID,
            motorInverted=True,
        )

        # Shot Calculator
        self.shotCalculator = ShotCalculator(
            drivetrain=self.robotDrive
        )

        # Indexer
        self.indexer = Indexer(
            motorCANID=IndexerConstants.kIndexerMotorID,
            motorInverted=True,
        )

        # Intake
        self.intake = Intake(
            motorCANID=IntakeConstants.kIntakeMotorCANID,
            motorInverted=False,
            solenoidModuleID=IntakeConstants.kSolenoidModuleID,
            pneumaticsModuleType=IntakeConstants.kPneumaticsModuleType,
            forwardChannel=IntakeConstants.kSolenoidForwardChannel,
            reverseChannel=IntakeConstants.kSolenoidReverseChannel
        )

        # Climber
        self.climber = Climber(
            motorCANID=ClimberConstants.kMotorID,
            motorInverted=ClimberConstants.kMotorInverted,
            solenoidCANID=ClimberConstants.kPCMID,
            pneumaticsModuleType=ClimberConstants.kPneumaticsModuleType,
            forwardChannel=ClimberConstants.kForwardChannel,
            reverseChannel=ClimberConstants.kReverseChannel,
            canCoderCANID=ClimberConstants.kCanCoderCANID,
            canCoderInverted=False
        )

        # Orchestra
        self.orchestra = OrchestraSubsystem(
            self.robotDrive,
            self.climber,
            self.shooter,
        )

        # Agitator
        self.agitator = Agitator(
            motorCANID=AgitatorConstants.kMotorCANID,
            motorInverted=AgitatorConstants.kMotorInverted
        )
 
        # Superstructure (MUST BE LAST)
        self.superstructure = Superstructure(
            drivetrain=self.robotDrive,
            shooter=self.shooter,
            shotCalculator=self.shotCalculator,
            indexer=self.indexer,
            agitator=self.agitator,
            intake=self.intake,
            climber=self.climber,
            vision=self.limelight,
            orchestra=self.orchestra,
            driverController=self.driverController,
            operatorController=self.operatorController
        )

        # Button Bindings

        manual = ManualClimb(
            self.superstructure,
            lambda: self.operatorController.getRawAxis(
                XboxController.Axis.kLeftY
            )
        )

        # Down (positive values)
        self.operatorController.axisGreaterThan(
            XboxController.Axis.kLeftY, 0.1
        ).whileTrue(manual)

        # Up (negative values)
        self.operatorController.axisLessThan(
            XboxController.Axis.kLeftY, -0.1
        ).whileTrue(manual)


        self.buttonBindings = ButtonBindings(self)
        self.buttonBindings.configureButtonBindings()

        # PathPlanner Named Commands
        NamedCommands.registerCommand("DeployIntake", self.superstructure.autoCreateStateCommand(RobotState.INTAKE_DEPLOYED))
        NamedCommands.registerCommand("RetractIntake", self.superstructure.autoCreateStateCommand(RobotState.INTAKE_RETRACTED))
        NamedCommands.registerCommand("StartIntake", self.superstructure.autoCreateStateCommand(RobotState.INTAKING))
        NamedCommands.registerCommand("PrepShot", self.superstructure.autoCreateStateCommand(RobotState.PREP_SHOT))
        NamedCommands.registerCommand("SetIdle", self.superstructure.autoCreateStateCommand(RobotState.IDLE))
        NamedCommands.registerCommand("ElevatorUp", self.superstructure.autoCreateStateCommand(RobotState.ELEVATOR_RISING))
        NamedCommands.registerCommand("ElevatorDown", self.superstructure.autoCreateStateCommand(RobotState.ELEVATOR_LOWERING))

        # PathPlanner Event Triggers
        EventTrigger("ElevatorUp").whileTrue(self.superstructure.autoCreateStateCommand(RobotState.ELEVATOR_RISING))

        # Auto Chooser

        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    # Autonomous

    def getAutonomousCommand(self) -> commands2.Command:
        command = self.autoChooser.getSelected()

        if command is None:
            print("WARNING: No autonomous routines selected!")
            return InstantCommand()

        print(f"Running autonomous routine: {command.getName()}")
        return command

    # Test Mode
    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        self.testChooser.setDefaultOption("None", None)
        return self.testChooser.getSelected()