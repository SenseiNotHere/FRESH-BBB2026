from commands2 import FunctionalCommand
from commands2.button import CommandGenericHID
from wpilib import SmartDashboard, Timer, XboxController
from wpilib.interfaces import GenericHID

from subsystems.shooter.shootersubsystem import Shooter
from subsystems.shooter.shot_calculator import ShotCalculator
from subsystems.intake.intakesubsystem import Intake
from subsystems.climber.climbersubsystem import Climber
from subsystems.shooter.indexersubsystem import Indexer
from subsystems.shooter.agitadorsubsystem import Agitator
from subsystems.vision.limelightcamera import LimelightCamera
from subsystems.drive.drivesubsystem import DriveSubsystem
from subsystems.orchestra.orchestrasubsystem import OrchestraSubsystem

from .robot_state import RobotState, RobotReadiness

from constants.constants import *
from constants.field_constants import *


class Superstructure:

    def __init__(
        self,
        drivetrain: DriveSubsystem | None = None,
        shooter: Shooter | None = None,
        indexer: Indexer | None = None,
        agitator: Agitator | None = None,
        shotCalculator: ShotCalculator | None = None,
        intake: Intake | None = None,
        climber: Climber | None = None,
        vision: LimelightCamera | None = None,
        orchestra: OrchestraSubsystem | None = None,
        driverController: CommandGenericHID | None = None,
        operatorController: CommandGenericHID | None = None,
    ):
        # Subsystems
        self.drivetrain = drivetrain
        self.shooter = shooter
        self.indexer = indexer
        self.agitator = agitator
        self.intake = intake
        self.climber = climber
        self.vision = vision
        self.orchestra = orchestra
        self.shotCalculator = shotCalculator
        self.driverController = driverController
        self.operatorController = operatorController

        # Subsystem availability flags
        self.hasShooter = self.shooter is not None
        self.hasIndexer = self.indexer is not None
        self.hasAgitator = self.agitator is not None
        self.hasIntake = self.intake is not None
        self.hasClimber = self.climber is not None
        self.hasVision = self.vision is not None
        self.hasOrchestra = self.orchestra is not None
        self.hasDriverController = self.driverController is not None
        self.hasOperatorController = self.operatorController is not None

        # State tracking
        self.robot_state = RobotState.IDLE
        self.robot_readiness = RobotReadiness()

        # Internal Timing / Debounce

        # Shooter spin-up debounce timer
        self._shooter_ready_since: float | None = None

        # Elevator "just reached target" detection
        self._prev_elevator_at_target = False

        # Controller rumble timer (auto stop)
        self._rumble_end_time: float | None = None

        self._state_handlers = {
            RobotState.IDLE: self._handle_idle,
            RobotState.INTAKING: self._handle_intaking,
            RobotState.INTAKING_AUTONOMOUS: self._handle_intaking,
            RobotState.PREP_SHOT: self._handle_prep_shot,
            RobotState.PREP_SHOT_AUTONOMOUS: self._handle_prep_shot,
            RobotState.SHOOTING: self._handle_shooting,
            RobotState.SHOOTING_AUTONOMOUS: self._handle_shooting,
            RobotState.ELEVATOR_RISING: self._handle_elevator_states,
            RobotState.ELEVATOR_LOWERING: self._handle_elevator_states,
            RobotState.AIRBREAK_ENGAGED_UP: self._handle_elevator_states,
            RobotState.AIRBREAK_ENGAGED_DOWN: self._handle_elevator_states,
            RobotState.ELEVATOR_MINIMUM: self._handle_elevator_states,
            RobotState.CLIMB_MANUAL: self._handle_climb_manual,
            RobotState.INTAKE_DEPLOYED: self._handle_intake_position,
            RobotState.INTAKE_RETRACTED: self._handle_intake_position,
            RobotState.PLAYING_SONG: self._handle_playing_song,
            RobotState.PLAYING_CHAMPIONSHIP_SONG: self._handle_playing_championship_song,
        }

        # Request flags
        self._request_intake = False
        self._request_shooter = False

    # Update Loop

    def update(self):
        """
        Superstructure update loop.
        Should be called periodically to update subsystem states.
        """

        SmartDashboard.putString("Superstructure/State", self.robot_state.name)

        self._update_readiness()

        handler = self._state_handlers.get(self.robot_state)
        if handler:
            handler()

        self._handle_music_cleanup()
        self._handle_rumble_timeout()

    # Readiness

    def _update_readiness(self):

        # Shooter readiness
        self.robot_readiness.shooterReady = self.shooter.atSpeed(tolerance_rpm=500)

        # Intake readiness
        self.robot_readiness.intakeDeployed = self.intake.isDeployed()

        # Climber readiness
        self.robot_readiness.elevatorAtHighTarget = self.climber.atHighTarget()
        self.robot_readiness.elevatorAtLowTarget = self.climber.atLowTarget()
        self.robot_readiness.elevatorAtClimbTarget = self.climber.atClimbTarget()

        # Feeding rule
        self.robot_readiness.canFeed = (
                self.robot_state in [
            RobotState.SHOOTING,
            RobotState.SHOOTING_AUTONOMOUS
        ]
                and self.robot_readiness.shooterReady
                and self.hasIndexer
        )

        # Rumble Controller - Readiness
        current = self.robot_readiness.elevatorAtHighTarget or self.robot_readiness.elevatorAtClimbTarget

        if current and not self._prev_elevator_at_target:
            self._rumble_controller(
                self.driverController,
                XboxController.RumbleType.kRightRumble,
                0.5
            )
            self._rumble_end_time = Timer.getFPGATimestamp() + 0.2

        self._prev_elevator_at_target = current

    # Public State API

    def createStateCommand(self, state: RobotState, finishImmediately=False):
        """
        Creates a FunctionalCommand that sets the robot state to the specified state.
        Handles intake and shooter stop/start based on the state.

        To be used outside superstructure.py.
        """

        def on_init():
            self.setState(state)

            if state in [
                RobotState.INTAKING,
                RobotState.INTAKING_AUTONOMOUS
            ]:
                self._request_intake = True

            if state in [
                RobotState.PREP_SHOT,
                RobotState.PREP_SHOT_AUTONOMOUS,
                RobotState.SHOOTING,
                RobotState.SHOOTING_AUTONOMOUS,
            ]:
                self._request_shooter = True

        def on_end(interrupted: bool):

            if state in [
                RobotState.INTAKING,
                RobotState.INTAKING_AUTONOMOUS
            ]:
                self.setState(RobotState.IDLE)
                self._request_intake = False

            elif state in [
                RobotState.PREP_SHOT,
                RobotState.PREP_SHOT_AUTONOMOUS,
                RobotState.SHOOTING,
                RobotState.SHOOTING_AUTONOMOUS
            ]:
                self.setState(RobotState.IDLE)
                self._request_shooter = False
            else:
                self.setState(RobotState.IDLE)

        return FunctionalCommand(
            on_init,
            lambda: None,
            on_end,
            lambda: finishImmediately,
            self.drivetrain,
        )

    def autoCreateStateCommand(self, state: RobotState):

        def init():
            print(f"AUTO COMMAND TRIGGERED -> {state}")
            self.setState(state)

        return FunctionalCommand(
            init,
            lambda: None,
            lambda interrupted: None,
            lambda: True,
            self.drivetrain,
        )

    def setState(self, newState: RobotState):
        """
        Sets the robot state to the specified state.

        To be used inside superstructure.py.
        """
        if newState == self.robot_state:
            return

        oldState = self.robot_state
        self.robot_state = newState

        # Reset shooter-ready
        self._shooter_ready_since = None

        # Rumble controller - States
        if newState in [
            RobotState.PREP_SHOT,
            RobotState.SHOOTING,
            RobotState.INTAKING,
        ]:
            self._rumble_controller(
                self.driverController,
                XboxController.RumbleType.kBothRumble,
                0.3
            )
            self._rumble_end_time = Timer.getFPGATimestamp() + 0.15

#        if newState == RobotState.ELEVATOR_RISING:
#            self._rumble_controller(
#                self.operatorController,
#                XboxController.RumbleType.kBothRumble,
#                0.3
#            )
#            self._rumble_end_time = Timer.getFPGATimestamp() + 0.2

#        if newState == RobotState.ELEVATOR_LOWERING:
#            self._rumble_controller(
#                self.operatorController,
#                XboxController.RumbleType.kBothRumble,
#                0.5
#            )
#            self._rumble_end_time = Timer.getFPGATimestamp() + 0.3

#        if newState == RobotState.ELEVATOR_MINIMUM:
#            self._rumble_controller(
#                self.operatorController,
#                XboxController.RumbleType.kBothRumble,
#                0.2
#            )
#            self._rumble_end_time = Timer.getFPGATimestamp() + 0.2

        print(f"[Superstructure] {oldState.name} -> {newState.name}")

        SmartDashboard.putString("Superstructure/State", newState.name)

    # State Handlers

    # Handles idle state by ensuring all subsystems are stopped.
    def _handle_idle(self):
        """
        Handles the idle state by stopping all subsystems.
        """
        self._stop_shooter()
        self._stop_indexer()
        self._stop_intake()
        self._stop_agitator()
        self._stop_orchestra()
        
    # Start intaking on entry
    def _handle_intaking(self):
        """
        Handles the intaking state by starting the intake.
        """
        pass

    # Start prep shot on entry
    def _handle_prep_shot(self):
        """
        Handles the prep shot state by spinning up the shooter and waiting for it to be ready.
        """
        pass

    # Start shooting on entry
    def _handle_shooting(self):
        """
        Handles the shooting state by spinning up the shooter and starting the indexer and agitator.
        """
        pass

    # Start elevator movement on entry
    def _handle_elevator_states(self):
        """
        !! STOPS ALL SUBSYSTEMS ON ENTRY !!

        Handles the elevator movement states by setting the climber target position accordingly.
        Handles the elevator airbreak states by releasing/engaging the airbrake as needed.
        """
        if not self.hasClimber:
            return

        self._stop_all_subsystems()

        state = self.robot_state

        # Max / Rise Height
        if state == RobotState.ELEVATOR_RISING:
            self.climber.releaseAirbrake()
            self.climber.setPosition(ClimberConstants.kRisenHeight)

        # Mid / Lower Height
        elif state == RobotState.ELEVATOR_LOWERING:
            self.climber.releaseAirbrake()
            self.climber.setPosition(ClimberConstants.kClimbedHeight)

            if self.climber.atClimbTarget():
                self.climber.engageAirbrake()

        # Minimum Height
        elif state == RobotState.ELEVATOR_MINIMUM:
            self.climber.releaseAirbrake()
            self.climber.setPosition(ClimberConstants.kMinPosition)

            if self.climber.atLowTarget():
                self.climber.engageAirbrake()

        # Airbrake engaged (up)
        elif state == RobotState.AIRBREAK_ENGAGED_UP:
            self.climber.engageAirbrake()

        # Airbrake engaged (down / debug)
        elif state == RobotState.AIRBREAK_ENGAGED_DOWN:
            self.climber.engageAirbrake()

    # Start manual climb on entry
    def _handle_climb_manual(self):
        """
        !! STOPS ALL SUBSYSTEMS ON ENTRY !!

        Handles the manual climb state by disabling all subsystems.
        Manual climb control is managed by the ManualClimb command.
        """
        if not self.hasClimber:
            return

        # Disable other mechanisms
        self._stop_all_subsystems()

        # DO NOT auto engage brake here.
        # ManualClimb command controls brake + movement.

    # Starts intake deployment/retraction on entry
    def _handle_intake_position(self):
        """
        Handles the intake position state by deploying/retracting the intake.
        """
        if not self.hasIntake:
            return

        # After executing, go back to IDLE
        self.setState(RobotState.IDLE)

    # Start song on entry
    def _handle_playing_song(self):
        """
        Handles the playing song state by playing the current loaded song.
        """
        self.orchestra.play_selected_song()

    # Start championship song on entry
    def _handle_playing_championship_song(self):
        """
        !! ONLY WHEN CHAMPIONSHIP IS ENABLED !!

        Handles the playing championship song state by playing the championship song.
        """
        self.orchestra.play_championship_song()

    # Helper Methods

    def _stop_shooter(self):
        pass

    def _stop_indexer(self):
        pass

    def _stop_agitator(self):
        pass

    def _stop_intake(self):
        pass

    def _stow_intake(self):
        pass

    def _stop_orchestra(self):
        if self.hasOrchestra:
            self.orchestra.stop()

    def _handle_music_cleanup(self):
        if self.robot_state != RobotState.PLAYING_SONG and self.hasOrchestra:
            self.orchestra.stop()

    def _stop_all_subsystems(self):
        self._stop_shooter()
        self._stop_indexer()
        self._stop_intake()
        self._stop_agitator()

    @staticmethod
    def _rumble_controller(
            controller,
            rumble_type: XboxController.RumbleType,
            rumble_value: float
    ):
        if controller is None:
            return

        controller.getHID().setRumble(
            rumble_type,
            rumble_value
        )

    def _handle_rumble_timeout(self):
        if not self._rumble_end_time:
            return

        if Timer.getFPGATimestamp() >= self._rumble_end_time:
            self._rumble_controller(
                self.driverController,
                XboxController.RumbleType.kBothRumble,
                0.0
            )
            self._rumble_controller(
                self.operatorController,
                XboxController.RumbleType.kBothRumble,
                0.0
            )
            self._rumble_end_time = None