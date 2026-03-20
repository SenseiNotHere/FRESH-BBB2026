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

from constants import *
from constants import *

from utils import log


class Superstructure(SuperstructureStates, SuperstructureHelpers):
    _instance = None

    def __init__(
            self,
            drivetrain: DriveSubsystem | None = None,
            intake: IntakeSubsystem | None = None,
            shooter: ShooterSubsystem | None = None,
            shooter2: ShooterSubsystem | None = None,
            indexer: IndexerSubsystem | None = None,
            agitator: AgitatorSubsystem | None = None,
            shotCalculator: ShotCalculator | None = None,
            vision: LimelightCamera | None = None,
            orchestra: OrchestraSubsystem | None = None,
            driverController: CommandGenericHID | None = None,
            operatorController: CommandGenericHID | None = None,
    ):
        """
        Superstructure.

        The Superstructure is the central coordination layer of the robot. It manages
        high-level robot states and orchestrates interactions between subsystems such
        as the drivetrain, intake, shooter, indexer, and agitator.

        Instead of subsystems directly controlling each other, the Superstructure
        defines robot-wide states (RobotState) and executes the appropriate subsystem
        logic for each state. This keeps subsystem logic isolated while allowing the
        robot to perform coordinated actions such as intaking, preparing a shot, and
        shooting.

        The Superstructure also tracks robot readiness conditions (RobotReadiness)
        which are used to determine when actions such as feeding or shooting are safe.

        This class is implemented as a single-instance subsystem and should be the
        final subsystem initialized in `RobotContainer`. Its update loop must be called
        periodically from `robotPeriodic()` to process state handlers and readiness
        logic.

        :param drivetrain: Drivetrain subsystem responsible for robot movement.
        :param intake: Intake subsystem used for game piece collection and positioning.
        :param shooter: Shooter subsystem responsible for spinning the shooter wheel.
        :param indexer: Indexer subsystem used to feed game pieces toward the shooter.
        :param agitator: Agitator subsystem used to assist with feeding game pieces.
        :param shotCalculator: Shot calculation subsystem used to compute distance-based shooter speeds and aiming information.
        :param vision: Limelight vision subsystem used for target tracking.
        :param orchestra: Orchestra subsystem used for Phoenix motor music playback.
        :param driverController: Driver controller used.
        :param operatorController: Operator controller used.
        """
        super().__init__()
        
        # Single instance
        if Superstructure._instance is not None:
            raise RuntimeError("Only one instance of Superstructure is allowed.")
        Superstructure._instance = self

        # Subsystems
        self.drivetrain = drivetrain
        self.intake = intake
        self.shooter = shooter
        self.shooter2 = shooter2
        self.indexer = indexer
        self.agitator = agitator
        self.vision = vision
        self.orchestra = orchestra
        self.shotCalculator = shotCalculator
        self.driverController = driverController
        self.operatorController = operatorController

        # Subsystem availability flags
        self.hasShooter = self.shooter is not None
        self.hasShooter2 = self.shooter2 is not None
        self.hasIntake = self.intake is not None
        self.hasIndexer = self.indexer is not None
        self.hasAgitator = self.agitator is not None
        self.hasShotCalc = self.shotCalculator is not None
        self.hasVision = self.vision is not None
        self.hasOrchestra = self.orchestra is not None
        self.hasDriverController = self.driverController is not None
        self.hasOperatorController = self.operatorController is not None
        # No flags for drivetrain cause without any of these, there's no robot :O

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
            shooter_ready = self.shooter.atSpeed(tolerance_rpm=50)

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
            onEnd=lambda interrupted: self.setState(RobotState.IDLE),
            isFinished=lambda: finishImmediately
        )

    def autoCreateStateCommand(self, state: RobotState):
        def on_init():
            log("Superstructure", f"AUTO COMMAND TRIGGERED -> {state}")
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

        log("Superstructure", f"{oldState.name} -> {newState.name}")

        SmartDashboard.putString("Superstructure/State", newState.name)

    # Public General API
    def getState(self):
        """
        :return: The current robot state.
        """
        return self.robot_state