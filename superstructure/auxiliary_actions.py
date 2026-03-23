from wpilib import DriverStation, Timer, XboxController, SmartDashboard
from utils import log


class AuxiliaryActions:
    def __init__(self, driverController=None):
        self.shiftNotifier = ShiftNotifier(driverController)

    def update(self):
        pass

class ShiftNotifier:
    def __init__(self, driverController=None):
        self.driverController = driverController

        self.matchStartTime = None

        # Debounce flags so alerts only fire once
        self._shift1_alert = False
        self._shift2_alert = False
        self._shift3_alert = False
        self._shift4_alert = False
        self._endgame_alert = False

        self._rumble_end_time = None

    def update(self):

        if DriverStation.isDisabled() and not DriverStation.isFMSAttached():
            self.matchStartTime = None
            return

        if not DriverStation.isTeleopEnabled():
            return

        if self.matchStartTime is None:
            self.matchStartTime = Timer.getFPGATimestamp()

        elapsed = Timer.getFPGATimestamp() - self.matchStartTime

        # SHIFT 1 ending in 5 seconds
        if elapsed > 5 and not self._shift1_alert:
            self._notify("SHIFT 1")
            self._shift1_alert = True

        # SHIFT 2 ending in 5 seconds
        if elapsed > 30 and not self._shift2_alert:
            self._notify("SHIFT 2")
            self._shift2_alert = True

        # SHIFT 3 ending in 5 seconds
        if elapsed > 55 and not self._shift3_alert:
            self._notify("SHIFT 3")
            self._shift3_alert = True

        # SHIFT 4 ending in 5 seconds
        if elapsed > 80 and not self._shift4_alert:
            self._notify("SHIFT 4")
            self._shift4_alert = True

        # ENDGAME starting in 5 seconds
        if elapsed > 105 and not self._endgame_alert:
            self._notify("ENDGAME")
            self._endgame_alert = True
    
        isAuto = DriverStation.isAutonomousEnabled()
        currentShift = ("AUTONOMOUS" if isAuto
        else "SHIFT 1" if (elapsed < 10 and not isAuto)
        else "SHIFT 2" if (elapsed < 35 and not isAuto)
        else "SHIFT 3" if (elapsed < 60 and not isAuto)
        else "SHIFT 4" if (elapsed < 85 and not isAuto)
        else "ENDGAME")
        SmartDashboard.putString('Current Shift', currentShift)

        self._handle_rumble_timeout()

    def _notify(self, text):
        log("Aux", f"{text}")

        if self.driverController is None:
            return

        try:
            self.driverController.getHID().setRumble(
                XboxController.RumbleType.kBothRumble,
                0.6
            )
            self._rumble_end_time = Timer.getFPGATimestamp() + 0.3
        except Exception as e:
            log("Aux", f"Failed to rumble: {e}")

    def _handle_rumble_timeout(self):
        if self._rumble_end_time is None or self.driverController is None:
            return

        if Timer.getFPGATimestamp() >= self._rumble_end_time:
            try:
                self.driverController.getHID().setRumble(
                    XboxController.RumbleType.kBothRumble,
                    0
                )
            except Exception as e:
                log("Aux", f"Failed to stop rumble: {e}")

            self._rumble_end_time = None