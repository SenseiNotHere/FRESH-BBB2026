from typing import TYPE_CHECKING
from wpilib import Timer
from .robot_state import RobotState
from constants.constants import ClimberConstants

if TYPE_CHECKING:
    from .superstructure import Superstructure

class SuperstructureStates:
    # Handles idle state by ensuring all subsystems are stopped.
    def _handle_idle(self: "Superstructure"):
        """
        Handles the idle state by stopping all subsystems.
        """
        self._stop_shooter()
        self._stop_indexer()
        self._stop_intake()
        self._stop_agitator()
        self._stop_orchestra()

    # Start intaking on entry
    def _handle_intaking(self: "Superstructure"):
        """
        Handles the intaking state by starting the intake.
        """

        if self.hasIntake:
            self.intake.intake()

    # Start prep shot on entry
    def _handle_prep_shot(self: "Superstructure"):
        """
        Handles the prep shot state by spinning up the shooter and waiting for it to be ready.
        """
        # Spin up shooter
        if self.hasShooter:
            if self.hasShotCalc:
                target_rps = self.shotCalculator.getTargetSpeedRPS()
                self.shooter.setTargetRPS(target_rps)
            else:
                self.shooter.useDashboardPercent()

        if self.hasIndexer:
            self.indexer.stop()
        if self.hasAgitator:
            self.agitator.stop()

        # Debounced transition to SHOOTING
        now = Timer.getFPGATimestamp()

        if self.robot_readiness.shooterReady:
            if self._shooter_ready_since is None:
                self._shooter_ready_since = now
            elif (now - self._shooter_ready_since) >= 0.12:
                self.setState(RobotState.SHOOTING)
        else:
            self._shooter_ready_since = None

    # Start shooting on entry
    def _handle_shooting(self: "Superstructure"):
        """
        Handles the shooting state by spinning up the shooter and starting the indexer and agitator.
        """
        # Keep spinning
        if self.hasShooter:
            if self.hasShotCalc:
                target_rps = self.shotCalculator.getTargetSpeedRPS()
                self.shooter.setTargetRPS(target_rps)
            else:
                self.shooter.useDashboardPercent()

        # Feed ONLY when ready
        if self.robot_readiness.canFeed:
            if self.hasIndexer:
                self.indexer.feed()
            if self.hasAgitator:
                self.agitator.feed()
        else:
            if self.hasIndexer:
                self.indexer.stop()
            if self.hasAgitator:
                self.agitator.stop()


    # Starts intake deployment/retraction on entry
    def _handle_intake_position(self: "Superstructure"):
        """
        Handles the intake position state by deploying/retracting the intake.
        """
        if not self.hasIntake:
            return

        # If intake isn't deployed, deploy it.
        if self.robot_state == RobotState.INTAKE_DEPLOYED:
            self.intake.deploy()

        # If intake is deployed, retract it.
        elif self.robot_state == RobotState.INTAKE_STOWED:
            self.intake.stow()

        # Go to IDLE after done.
        self.setState(RobotState.IDLE)

    # Starts intake and shooter on entry, when shooterReady start indexer and agitator
    def _handle_passing_fuel(self: "Superstructure"):
        """
        Handles the event of passing fuel by starting the Intake, then starting the Shooter (as in PREP_SHOT).
        Indexer and Agitator start running after the shooter is ready.
        """
        # Start intake.
        if self.hasIntake:
            self.intake.intake()

        # Start shooter.
        if self.hasShooter:
            self.shooter.useDashboardPercent()

        # Start indexer and agitator when shooter is ready.
        if self.robot_readiness.shooterReady:
            if self.hasIndexer:
                self.indexer.feed()
            if self.hasAgitator:
                self.agitator.feed()

    # Start song on entry
    def _handle_playing_song(self: "Superstructure"):
        """
        Handles the playing song state by playing the current loaded song.
        """
        if self.hasOrchestra:
            self.orchestra.play_selected_song()

    # Start championship song on entry
    def _handle_playing_championship_song(self: "Superstructure"):
        """
        !! ONLY WHEN CHAMPIONSHIP IS ENABLED !!

        Handles the playing championship song state by playing the championship song.
        """
        if self.hasOrchestra:
            self.orchestra.play_championship_song()
