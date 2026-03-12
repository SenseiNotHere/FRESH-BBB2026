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

    def _stop_intake(self: "Superstructure"):
        if self.hasIntake:
            self.intake.stop_intake()

    def _stop_indexer(self: "Superstructure"):
        if self.hasIndexer:
            self.indexer.stop()

    def _stop_agitator(self: "Superstructure"):
        if self.hasAgitator:
            self.agitator.stop()

    def _stop_orchestra(self: "Superstructure"):
        if self.hasOrchestra:
            self.orchestra.stop()

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
