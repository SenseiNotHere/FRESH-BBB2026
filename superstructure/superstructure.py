from commands2 import FunctionalCommand
from commands2.button import CommandGenericHID
from wpilib import SmartDashboard, Timer, XboxController
from wpilib.interfaces import GenericHID

from subsystems import (
    IntakeSubsystem,
    ShooterSubsystem,
    ShotCalculator,
    IndexerSubsystem,
    AgitatorSubsystem,
    LimelightCamera,
    DriveSubsystem,
    OrchestraSubsystem
)

from .robot_state import RobotState, RobotReadiness, ReadinessList
from .auxiliary_actions import AuxiliaryActions
from .superstructure_helpers import SuperstructureHelpers
from .superstructure_states import SuperstructureStates

from constants.constants import *
from constants.field_constants import *


class Superstructure(SuperstructureStates, SuperstructureHelpers):
    def __init__(
            self,
            drivetrain: DriveSubsystem | None = None,
            intake: IntakeSubsystem | None = None,
            shooter: ShooterSubsystem | None = None,
            indexer: IndexerSubsystem | None = None,
            agitator: AgitatorSubsystem | None = None,
            shotCalculator: ShotCalculator | None = None,
            vision: LimelightCamera | None = None,
            orchestra: OrchestraSubsystem | None = None,
            driverController: CommandGenericHID | None = None,
            operatorController: CommandGenericHID | None = None,
    ):
        # Subsystems
        self.drivetrain = drivetrain
        self.intake = intake
        self.shooter = shooter
        self.indexer = indexer
        self.agitator = agitator
        self.vision = vision
        self.orchestra = orchestra
        self.shotCalculator = shotCalculator
        self.driverController = driverController
        self.operatorController = operatorController

        # Subsystem availability flags
        self.hasShooter = self.shooter is not None
        self.hasIntake = self.intake is not None
        self.hasIndexer = self.indexer is not None
        self.hasAgitator = self.agitator is not None
        self.hasShotCalc = self.shotCalculator is not None
        self.hasVision = self.vision is not None
        self.hasOrchestra = self.orchestra is not None
        self.hasDriverController = self.driverController is not None
        self.hasOperatorController = self.operatorController is not None

        # State tracking
        self.robot_state = RobotState.IDLE
        self.robot_readiness = RobotReadiness()

        # State to State-Handler
        self._state_handlers = {
            RobotState.IDLE: self._handle_idle,

            # Intake
            RobotState.INTAKING: self._handle_intaking,
            RobotState.INTAKING_AUTONOMOUS: self._handle_intaking,
            RobotState.INTAKE_DEPLOYED: self._handle_intake_position,
            RobotState.INTAKE_STOWED: self._handle_intake_position,

            # Shooter
            RobotState.PREP_SHOT: self._handle_prep_shot,
            RobotState.PREP_SHOT_AUTONOMOUS: self._handle_prep_shot,
            RobotState.SHOOTING: self._handle_shooting,
            RobotState.SHOOTING_AUTONOMOUS: self._handle_shooting,

            # Music
            RobotState.PLAYING_SONG: self._handle_playing_song,
            RobotState.PLAYING_CHAMPIONSHIP_SONG: self._handle_playing_championship_song,

            # Misc
            RobotState.PASSING_FUEL: self._handle_passing_fuel
        }

        # Internal Timing / Debounce
        # Shooter spin-up debounce timer
        self._shooter_ready_since: float | None = None

        # Auxiliary actions
        self.auxiliary_actions = AuxiliaryActions(self.driverController)

        # Controller rumble
        self._rumble_end_time = None

        # State start time
        self._state_start_time = Timer.getFPGATimestamp()

    # Update Loop

    def update(self):
        """
        Superstructure update loop.
        Should be called periodically to update subsystem states.
        
        Called by robotPeriodic() in robot.py
        """

        SmartDashboard.putString("Superstructure/State", self.robot_state.name)

        self._update_readiness()

        handler = self._state_handlers.get(self.robot_state)
        if handler:
            handler()

        self._handle_music_cleanup()
        self._handle_rumble_timeout()

        # Auxiliary actions
        self.auxiliary_actions.update()

    # Readiness

    def _update_readiness(self):

        # Shooter
        shooter_ready = False
        if self.hasShooter:
            shooter_ready = self.shooter.atSpeed(tolerance_rpm=100)

        self.robot_readiness.shooterReady = shooter_ready

        # Intake
        intake_deployed = False
        if self.hasIntake:
            intake_deployed = self.intake.is_deployed()

        self.robot_readiness.intakeDeployed = intake_deployed

        # Feeding
        shooting_states = {
            RobotState.SHOOTING,
            RobotState.SHOOTING_AUTONOMOUS
        }
        can_feed = (
                self.robot_state in shooting_states
                and shooter_ready
                and self.hasIndexer
        )

        self.robot_readiness.canFeed = can_feed

    # Public Superstructure API
    def createStateCommand(self, state: RobotState, finishImmediately=False):
        """
        Creates a command that sets the robot state to the specified state.
        """
        def on_end(interrupted):
            if self.robot_state == state:
                self.setState(RobotState.IDLE)

        return FunctionalCommand(
            onInit=lambda: self.setState(state),
            onExecute=lambda: None,
            onEnd=on_end,
            isFinished=lambda: finishImmediately
        )

    def autoCreateStateCommand(self, state: RobotState):
        def on_init():
            print(f"AUTO COMMAND TRIGGERED -> {state}")
            self.setState(state)

        return FunctionalCommand(
            onInit=on_init,
            onExecute=lambda: None,
            onEnd=lambda interrupted: None,
            isFinished=lambda: True,
        )

    def setState(self, newState: RobotState, force: bool = False):
        """
        Sets the robot state to the specified state.

        To be used inside superstructure.py.
        
        :newState: The new state to set (RobotState enum).
        :force: Whether to force the state change even if it's the same as the current state. (Great for restarting a state)
        """
        if not force and newState == self.robot_state:
            return

        oldState = self.robot_state
        self.robot_state = newState

        # Timer
        self._state_start_time = Timer.getFPGATimestamp()

        # Reset shooter-ready
        self._shooter_ready_since = None

        print(f"[Superstructure] {oldState.name} -> {newState.name}")

        SmartDashboard.putString("Superstructure/State", newState.name)

    # Public General API
    def getState(self):
        """
        :return: The current robot state.
        """
        return self.robot_state