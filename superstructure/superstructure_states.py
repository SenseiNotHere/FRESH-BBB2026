from typing import TYPE_CHECKING
from wpilib import Timer
from .robot_state import RobotState

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
        self._stop_agitator()
        self._stop_orchestra()

    # Start intaking on entry
    def _handle_intaking(self: "Superstructure"):
        """
        Handles the intaking state by starting the intake.
        """
        pass

    def _handle_prep_shot(self: "Superstructure"):
        """
        Handles the prep shot state by spinning up the shooter and waiting for it to be ready.
        """
        self._spin_up_shooters()
        self._stop_feeders()

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
        Handles the shooting state by spinning up the shooter and starting the indexers and agitator.
        """
        self._spin_up_shooters()

        if self.robot_readiness.canFeed:
            self._feed_shooters()
        else:
            self._stop_feeders()

    # Starts intake deployment/retraction on entry
    def _handle_intake_position(self: "Superstructure"):
        """
        Handles the intake position state by deploying/retracting the intake.
        """
        pass

    # Starts intake and shooter on entry, when shooterReady start indexers and agitator
    def _handle_passing_fuel(self: "Superstructure"):
        """
        Handles passing fuel by running intake, spinning up shooters from dashboard percent,
        and feeding only once the shooter path is ready.
        """
        pass

        self._spin_up_shooters_dashboard()

        if self.robot_readiness.shooterReady:
            self._feed_shooters()
        else:
            self._stop_feeders()

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
