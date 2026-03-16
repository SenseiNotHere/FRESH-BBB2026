from commands2 import Subsystem
from wpilib import SmartDashboard, SendableChooser, Timer

from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    ResetMode,
    PersistMode
)

class AgitatorSubsystem(Subsystem):
    def __init__(
            self,
            motorCANID: int,
            motorInverted: bool,
            motor2CANID: int | None = None,
            motor2Inverted: bool = False
    ):
        """
        Agitator Subsystem.

        This subsystem controls the agitator mechanism using one or two motors.
        Each motor has its own speed chooser on the dashboard.

        Hardware:
        - Spark MAX(s) controlling brushless motor(s) used to drive the agitator.

        :param motorCANID: CAN ID of the Spark MAX controlling the first agitator motor.
        :param motorInverted: Whether the first agitator motor is inverted.
        :param motor2CANID: (Optional) CAN ID of the Spark MAX controlling the second agitator motor.
        :param motor2Inverted: Whether the second agitator motor is inverted.
        """
        super().__init__()

        # Motor 1 Setup
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
        SmartDashboard.putData("Agitator/Motor 1 Speed Chooser", self.speedChooser)

        # Motor 2 Setup (Optional)
        self.motor2 = None
        self.speedChooser2 = None
        self.hasMotor2 = motor2CANID is not None

        if self.hasMotor2:
            self.motor2 = SparkMax(motor2CANID, SparkLowLevel.MotorType.kBrushless)
            motor2Config = SparkMaxConfig()
            motor2Config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
            motor2Config.inverted(motor2Inverted)

            self.motor2.configure(
                motor2Config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )

            self.speedChooser2 = SendableChooser()
            self.speedChooser2.setDefaultOption("25%", 0.25)
            self.speedChooser2.addOption("5%", 0.05)
            self.speedChooser2.addOption("50%", 0.5)
            self.speedChooser2.addOption("75%", 0.75)
            self.speedChooser2.addOption("100%", 1.0)
            SmartDashboard.putData("Agitator/Motor 2 Speed Chooser", self.speedChooser2)

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

        SmartDashboard.putNumber("Agitator/Motor 1 Speed", self.motor.get())
        if self.hasMotor2:
            SmartDashboard.putNumber("Agitator/Motor 2 Speed", self.motor2.get())

        SmartDashboard.putBoolean("Agitator/Agitator Running", self.isRunning())
        SmartDashboard.putBoolean("Agitator/Agitator Oscillating", self._oscillateEnabled)
        SmartDashboard.putBoolean("Agitator/Agitator Forward", self._forward)

    def feed(self) -> None:
        self._oscillateEnabled = False
        self.motor.set(self.speedChooser.getSelected())
        if self.hasMotor2:
            self.motor2.set(self.speedChooser2.getSelected())

    def reverse(self) -> None:
        self._oscillateEnabled = False
        self.motor.set(-self.speedChooser.getSelected())
        if self.hasMotor2:
            self.motor2.set(-self.speedChooser2.getSelected())

    def stop(self) -> None:
        self._oscillateEnabled = False
        self._forward = True
        self._lastToggleTime = 0.0
        self.motor.set(0)
        if self.hasMotor2:
            self.motor2.set(0)

    def isRunning(self) -> bool:
        running = abs(self.motor.get()) > 0.01
        if self.hasMotor2:
            running = running or abs(self.motor2.get()) > 0.01
        return running

    def startOscillate(self, forwardSeconds: float = 2.0, backwardSeconds: float = 0.5) -> None:
        self._forwardPeriod = max(0.05, float(forwardSeconds))
        self._backwardPeriod = max(0.05, float(backwardSeconds))
        if not self._oscillateEnabled:  # Only reset timer if starting fresh
            self._lastToggleTime = Timer.getFPGATimestamp()
            self._forward = True
        self._oscillateEnabled = True
        self._applyOscillateOutput()  # forward immediately

    def _applyOscillateOutput(self) -> None:
        speed1 = self.speedChooser.getSelected()
        self.motor.set(speed1 if self._forward else -speed1)

        if self.hasMotor2:
            speed2 = self.speedChooser2.getSelected()
            self.motor2.set(speed2 if self._forward else -speed2)