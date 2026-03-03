from commands2 import Subsystem
from wpilib import SmartDashboard, SendableChooser, Timer

from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    ResetMode,
    PersistMode
)

class Agitator(Subsystem):
    def __init__(self, motorCANID: int, motorInverted: bool):
        super().__init__()

        self.motor = SparkMax(motorCANID, SparkLowLevel.MotorType.kBrushless)

        motorConfig = SparkMaxConfig()
        motorConfig.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        motorConfig.inverted(motorInverted)

        self.motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("25%", 0.25)
        self.speedChooser.addOption("5%", 0.05)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("75%", 0.75)
        self.speedChooser.addOption("100%", 1.0)
        SmartDashboard.putData("Agitator Chooser", self.speedChooser)

        # Oscillation logic
        self._oscillateEnabled = False
        self._forwardPeriod = 2.0
        self._backwardPeriod = 0.5
        self._lastToggleTime = 0.0
        self._forward = True  # forward first

    def periodic(self):
        # Run the timed flip logic if enabled
        if self._oscillateEnabled:
            now = Timer.getFPGATimestamp()
            elapsed = now - self._lastToggleTime
            # Use different periods for forward vs backward
            period = self._forwardPeriod if self._forward else self._backwardPeriod
            if elapsed >= period:
                self._lastToggleTime = now
                self._forward = not self._forward
            self._applyOscillateOutput()
        SmartDashboard.putNumber("Agitator Speed", self.motor.get())
        SmartDashboard.putBoolean("Agitator Running", self.isRunning())
        SmartDashboard.putBoolean("Agitator Oscillating", self._oscillateEnabled)
        SmartDashboard.putBoolean("Agitator Forward", self._forward)

    def feed(self) -> None:
        self._oscillateEnabled = False
        self.motor.set(self.speedChooser.getSelected())

    def reverse(self) -> None:
        self._oscillateEnabled = False
        self.motor.set(-self.speedChooser.getSelected())

    def stop(self) -> None:
        self._oscillateEnabled = False
        self._forward = True
        self._lastToggleTime = 0.0
        self.motor.set(0)

    def isRunning(self) -> bool:
        return abs(self.motor.get()) > 0.01

    def startOscillate(self, forwardSeconds: float = 2.0, backwardSeconds: float = 0.5) -> None:
        self._forwardPeriod = max(0.05, float(forwardSeconds))
        self._backwardPeriod = max(0.05, float(backwardSeconds))
        if not self._oscillateEnabled:  # Only reset timer if starting fresh
            self._lastToggleTime = Timer.getFPGATimestamp()
            self._forward = True
        self._oscillateEnabled = True
        self._applyOscillateOutput()  # forward immediately

    def _applyOscillateOutput(self) -> None:
        speed = self.speedChooser.getSelected()
        self.motor.set(speed if self._forward else -speed)