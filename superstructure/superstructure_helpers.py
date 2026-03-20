from typing import TYPE_CHECKING
from commands2.button import CommandGenericHID
from wpilib import Timer, XboxController
from .robot_state import RobotState

if TYPE_CHECKING:
    from .superstructure import Superstructure

class SuperstructureHelpers:
    def _stop_shooter(self: "Superstructure"):
        if self.hasShooter:
            self.shooter.stop()
        if self.hasShooter2:
            self.shooter2.stop()

    def _stop_intake(self: "Superstructure"):
        if self.hasIntake:
            self.intake.stop_intake()

    def _stop_indexer(self: "Superstructure"):
        if self.hasIndexer:
            self.indexer.stop()

    def _stop_intake_position(self: "Superstructure"):
        if self.hasIntake:
            self.intake.stop_deploy()

    def _stop_agitator(self: "Superstructure"):
        if self.hasAgitator:
            self.agitator.stop()

    def _stop_orchestra(self: "Superstructure"):
        if self.hasOrchestra:
            self.orchestra.stop()

    def _spin_up_shooters(self: "Superstructure"):
        if not self.hasShooter and not self.hasShooter2:
            return

        target_rps = None
        if self.hasShotCalc:
            target_rps = self.shotCalculator.getTargetSpeedRPS()

        if self.hasShooter:
            if target_rps is not None:
                self.shooter.setTargetRPS(target_rps)
            else:
                self.shooter.useDashboardPercent()

        if self.hasShooter2:
            if target_rps is not None:
                self.shooter2.setTargetRPS(target_rps)
            else:
                self.shooter2.useDashboardPercent()

    def _stop_feeders(self: "Superstructure"):
        if self.hasIndexer:
            self.indexer.stop()

        if self.hasAgitator:
            self.agitator.stop()

    def _spin_up_shooters_dashboard(self: "Superstructure"):
        if self.hasShooter:
            self.shooter.useDashboardPercent()
        if self.hasShooter2:
            self.shooter2.useDashboardPercent()

    def _feed_shooters(self: "Superstructure"):
        if self.hasIndexer:
            self.indexer.feed()

        if self.hasAgitator:
            self.agitator.feed()

    def _handle_music_cleanup(self: "Superstructure"):
        if self.robot_state != RobotState.PLAYING_SONG and self.hasOrchestra:
            self.orchestra.stop()

    def _stop_all_subsystems(self: "Superstructure"):
        self._stop_shooter()
        self._stop_indexer()
        self._stop_agitator()

    @staticmethod
    def _rumble_controller(
            controller: CommandGenericHID | None,
            rumble_type: XboxController.RumbleType,
            rumble_value: float
    ):
        if controller is None:
            return

        controller.getHID().setRumble(
            rumble_type,
            rumble_value
        )

    def _handle_rumble_timeout(self: "Superstructure"):
        if not self._rumble_end_time:
            return

        if Timer.getFPGATimestamp() >= self._rumble_end_time:
            if self.hasDriverController:
                self._rumble_controller(
                    self.driverController,
                    XboxController.RumbleType.kBothRumble,
                    0.0
                )
            self._rumble_end_time = None
