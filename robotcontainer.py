from __future__ import annotations

import typing

import commands2
from commands2 import InstantCommand
from commands2.button import CommandGenericHID
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

from buttonbindings import ButtonBindings
from commands import HolonomicDrive
from constants.constants import (
    OIConstants,
    ShooterConstants,
    IndexerConstants,
    IntakeConstants,
    AgitatorConstants
)

from superstructure import Superstructure

from subsystems import (
    DriveSubsystem,
    BadSimPhysics,
    AutonomousSubsystem,
    AutoBuilder,
    IntakeSubsystem,
    OrchestraSubsystem,
    AgitatorSubsystem,
    IndexerSubsystem,
    ShooterSubsystem,
    ShotCalculator,
    LimelightCamera,
    LimelightLocalizer,
)

from utils import log, print_banner

class RobotContainer:
    """
    The container for the robot. Subsystems are initialized here,
    button bindings are set up, and auto chooser is sent to the dashboard.
    """

    def __init__(self, robot):

        print_banner("ROBOT INITIALIZATION STARTING")

        # Drive Subsystem
        log("RobotContainer","Initializing DriveSubsystem...")
        self.robotDrive = DriveSubsystem()

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)

        log("Robot Container", "DriveSubsystem Initialized!")

        # Test Chooser
        log("Robot Container","Initializing Test Chooser...")
        self.testChooser = SendableChooser()
        SmartDashboard.putData("Test Chooser", self.testChooser)
        log("Robot Container","Test Chooser Initialized!")

        # Controllers
        log("Robot Container","Initializing Controllers...")
        self.driverController = CommandGenericHID(
            OIConstants.kDriverControllerPort
        )
        self.operatorController = CommandGenericHID(
            OIConstants.kOperatorControllerPort
        )
        log("Robot Container","Controllers Initialized!")

        # Vision / Localization
        log("Robot Container","Initializing Vision...")
        self.localizer = LimelightLocalizer(
            drivetrain=self.robotDrive,
            flipIfRed=False,
        )

        self.limelight = LimelightCamera("limelight-front")
        self.limelight.setPiPMode(1)

        self.limelightBack = LimelightCamera("limelight-back")

        self.localizer.addCamera(
            camera=self.limelight,
            cameraPoseOnRobot=Translation3d(-0.17, 0.0, 0.508),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(180),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )

        self.localizer.addCamera(
            camera=self.limelightBack,
            cameraPoseOnRobot=Translation3d(-0.17, -0.165, 0.406),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(270),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )

        log("Robot Container","Vision Initialized!")

        # Default Drive Command
        log("Robot Container","Setting Default Drive Command...")
        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: -self.driverController.getRawAxis(
                    XboxController.Axis.kLeftY
                ),
                leftSpeed=lambda: -self.driverController.getRawAxis(
                    XboxController.Axis.kLeftX
                ),
                rotationSpeed=lambda: self.driverController.getRawAxis(
                    XboxController.Axis.kRightX
                ),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=False,
                square=True,
            )
        )
        log("Robot Container","Default Drive Command Set!")

        # Intake
        log("Robot Container","Initializing Intake...")
        self.intake = IntakeSubsystem(
            deployMotorCANID=IntakeConstants.kDeployMotorID,
            deployMotorInverted=IntakeConstants.kDeployMotorInverted,
            intakeMotorCANID=IntakeConstants.kIntakeMotorID,
            intakeMotorInverted=IntakeConstants.kIntakeMotorInverted,
        )
        log("Robot Container","Intake Initialized!")

        # Shooter
        log("Robot Container","Initializing Shooter...")
        self.shooter = ShooterSubsystem(
            motorCANID=ShooterConstants.kShooterMotorID,
            motorInverted=True,
        )
        log("Robot Container","Shooter Initialized!")

        # Shot Calculator
        log("Robot Container","Initializing Shot Calculator...")
        self.shotCalculator = ShotCalculator(
            drivetrain=self.robotDrive
        )
        log("Robot Container","Shot Calculator Initialized!")

        # Indexer
        log("Robot Container","Initializing Indexer...")
        self.indexer = IndexerSubsystem(
            motorCANID=IndexerConstants.kIndexerMotorID,
            motorInverted=True,
        )
        log("Robot Container","Indexer Initialized!")

        # Orchestra
        log("Robot Container","Initializing Orchestra...")
        self.orchestra = OrchestraSubsystem(
            self.robotDrive,
            self.shooter,
        )
        log("Robot Container","Orchestra Initialized!")

        # Agitator
        log("Robot Container","Initializing Agitator...")
        self.agitator = AgitatorSubsystem(
            motorCANID=AgitatorConstants.kMotorCANID,
            motorInverted=AgitatorConstants.kMotorInverted,
        )
        log("Robot Container", "Agitator Initialized!")

        # Superstructure (MUST BE LAST)
        log("Robot Container","Initializing Superstructure...")
        self.superstructure = Superstructure(
            drivetrain=self.robotDrive,
            intake=self.intake,
            shooter=self.shooter,
            indexer=self.indexer,
            agitator=self.agitator,
            shotCalculator=self.shotCalculator,
            vision=self.limelight,
            orchestra=self.orchestra,
            driverController=self.driverController,
            operatorController=self.operatorController,
        )
        log("Robot Container","Superstructure Initialized!")

        # Autonomous
        log("Robot Container","Initializing Autonomous...")
        self.autonomousSubsystem = AutonomousSubsystem(self.robotDrive, self)

        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        log("Robot Container","Autonomous Initialized!")

        # Button Bindings
        log("Robot Container","Initializing Button Bindings...")
        self.buttonBindings = ButtonBindings(self)
        self.buttonBindings.configureButtonBindings()
        log("Robot Container","Button Bindings Initialized!")

        print_banner("ROBOT INITIALIZATION COMPLETE")

        # FMS Stuff
        isFMSAttached = DriverStation.isFMSAttached()
        log("Robot Container","Current Robot Status:")

        if isFMSAttached:
            log("Robot Container","FMS is attached! Welcome to the competition field, team 1811!")
            log("Robot Container","Bobby the B Box is ready to rock and roll!")
        else:
            log("Robot Container","FMS is not attached! Running in practice mode.")
            log("Robot Container","Bobby the B Box is alive!")

            # Rest of Auto stuff
            self._lastPreviewedAuto = None

    def updateAutoPreview(self):
        selected = self.autoChooser.getSelected()

        if selected != self._lastPreviewedAuto:
            self.autonomousSubsystem.drawAuto(selected)
            self._lastPreviewedAuto = selected

    # Autonomous
    def getAutonomousCommand(self) -> commands2.Command:
        command = self.autoChooser.getSelected()

        if command is None:
            log("Autonomous", "WARNING: No autonomous routines selected!")
            return InstantCommand()

        log("Autonomous",f"Running autonomous routine: {command.getName()}")
        return command

    # Test Mode
    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        self.testChooser.setDefaultOption("None", None)
        return self.testChooser.getSelected()