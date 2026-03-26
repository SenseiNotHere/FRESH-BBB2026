from __future__ import annotations

from typing import Optional

from commands2 import InstantCommand, Command, TimedCommandRobot
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
from constants import (
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
    
    Dictionary:
        - self.vroomvroom = DriveSubsystem
        - self.gulp = IntakeSubsystem
        - self.pew = ShooterSubsystem
        - self.pewpew = ShooterSubsystem 2
        - self.calc = ShotCalculator
        - self.slurp = IndexerSubsystem
        - self.lavadora = AgitatorSubsystem
        - self.lemon = LimelightCamera
        - self.limao = LimelightCamera 2
        - self.orca = OrchestraSubsystem
        - self.vroomvroomController = Driver Controller
        - self.statesideController = Operator Controller
        - self.megamente = Superstructure
    """

    def __init__(self, robot):

        print_banner("ROBOT INITIALIZATION STARTING")

        # Controllers
        log("Robot Container","Initializing Controllers...")
        self.vroomvroomController = CommandGenericHID(
            OIConstants.kDriverControllerPort
        )
        self.statesideController = CommandGenericHID(
            OIConstants.kOperatorControllerPort
        )
        log("Robot Container","Controllers Initialized!")

        # Drive Subsystem
        log("RobotContainer","Initializing DriveSubsystem...")

        def slowdown_when():
            return 0.5 if self.vroomvroomController.getRawAxis(XboxController.Axis.kLeftTrigger) < 0.5 else 1.0

        self.vroomvroom = DriveSubsystem(maxSpeedScaleFactor=slowdown_when)

        if TimedCommandRobot.isSimulation():
            self.vroomvroom.simPhysics = BadSimPhysics(self.vroomvroom, robot)

        log("Robot Container", "DriveSubsystem Initialized!")

        # Test Chooser
        log("Robot Container","Initializing Test Chooser...")
        self.testChooser = SendableChooser()
        SmartDashboard.putData("Test Chooser", self.testChooser)
        log("Robot Container","Test Chooser Initialized!")

        # Vision / Localization
        log("Robot Container","Initializing Vision...")
        self.localizer = LimelightLocalizer(
            drivetrain=self.vroomvroom,
            flipIfRed=False,
        )

        self.lemon = LimelightCamera("limelight-front")
        self.lemon.setPiPMode(1)

        self.limao = LimelightCamera("limelight-back")

        self.localizer.addCamera(
            camera=self.lemon,
            cameraPoseOnRobot=Translation3d(-0.17, 0.0, 0.508),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(180),
            minPercentFrame=0.07,
            maxRotationSpeed=720,
        )

        log("Robot Container","Vision Initialized!")

        # Default Drive Command
        log("Robot Container","Setting Default Drive Command...")
        self.vroomvroom.setDefaultCommand(
            HolonomicDrive(
                self.vroomvroom,
                forwardSpeed=lambda: -self.vroomvroomController.getRawAxis(
                    XboxController.Axis.kLeftY
                ),
                leftSpeed=lambda: -self.vroomvroomController.getRawAxis(
                    XboxController.Axis.kLeftX
                ),
                rotationSpeed=lambda: self.vroomvroomController.getRawAxis(
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
        self.gulp = IntakeSubsystem(
            deployMotorCANID=IntakeConstants.kDeployMotorID,
            deployMotorInverted=IntakeConstants.kDeployMotorInverted,
            intakeMotorCANID=IntakeConstants.kIntakeMotorID,
            intakeMotorInverted=IntakeConstants.kIntakeMotorInverted,
        )
        log("Robot Container","Intake Initialized!")

        # Shooters
        log("Robot Container","Initializing Shooters...")
        self.pew = ShooterSubsystem(
            motorCANID=ShooterConstants.kShooterMotorID,
            motorInverted=ShooterConstants.kShooterMotorInverted,
        )
        self.pewpew = ShooterSubsystem(
            motorCANID=ShooterConstants.kShooterMotor2ID,
            motorInverted=ShooterConstants.kShooterMotor2Inverted,
        )
        
        log("Robot Container","Shooters Initialized!")

        # Shot Calculator
        log("Robot Container","Initializing Shot Calculator...")
        self.calc = ShotCalculator(
            drivetrain=self.vroomvroom
        )
        log("Robot Container","Shot Calculator Initialized!")

        # Indexer
        log("Robot Container","Initializing Indexer...")
        self.slurp = IndexerSubsystem(
            motorCANID=IndexerConstants.kIndexerMotorID,
            motorInverted=IndexerConstants.kIndexerMotorInverted
        )
        log("Robot Container","Indexer Initialized!")

        # Orchestra
        log("Robot Container","Initializing Orchestra...")
        self.orca = OrchestraSubsystem(
            self.vroomvroom,
            self.pew,
        )
        log("Robot Container","Orchestra Initialized!")

        # Agitator
        log("Robot Container","Initializing Agitator...")
        self.lavadora = AgitatorSubsystem(
            motorCANID=AgitatorConstants.kMotorCANID,
            motorInverted=AgitatorConstants.kMotorInverted,
        )
        log("Robot Container", "Agitator Initialized!")

        # Superstructure (MUST BE LAST)
        log("Robot Container","Initializing Superstructure...")
        self.megamente = Superstructure(
            drivetrain=self.vroomvroom,
            intake=self.gulp,
            shooter=self.pew,
            shooter2=self.pewpew,
            indexer=self.slurp,
            agitator=self.lavadora,
#            shotCalculator=self.calc,
            vision=self.lemon,
            orchestra=self.orca,
            driverController=self.vroomvroomController,
            operatorController=self.statesideController,
        )
        log("Robot Container","Superstructure Initialized!")

        # Autonomous
        log("Robot Container","Initializing Autonomous...")
        self.autonomousSubsystem = AutonomousSubsystem(self.vroomvroom, self)

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
            log("Robot Container","Bobby the B Box II is ready to rock and roll!")
        else:
            log("Robot Container","FMS is not attached! Running in practice mode.")
            log("Robot Container","Bobby the B Box II is alive!")

            # Rest of Auto stuff
            self._lastPreviewedAuto = None

    def updateAutoPreview(self):
        selected = self.autoChooser.getSelected()

        if selected != self._lastPreviewedAuto:
            self.autonomousSubsystem.drawAuto(selected.getName())
            self._lastPreviewedAuto = selected

    # Autonomous
    def getAutonomousCommand(self) -> Command:
        command = self.autoChooser.getSelected()

        if command is None:
            log("Autonomous", "WARNING: No autonomous routines selected!")
            return InstantCommand()

        log("Autonomous",f"Running autonomous routine: {command.getName()}")
        return command

    # Test Mode
    def getTestCommand(self) -> Optional[Command]:
        self.testChooser.setDefaultOption("None", None)
        return self.testChooser.getSelected()