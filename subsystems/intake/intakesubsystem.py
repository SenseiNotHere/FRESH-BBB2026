from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation, Timer

from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage
from phoenix6.configs import TalonFXConfiguration, Slot0Configs
from phoenix6.signals import NeutralModeValue, InvertedValue

from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkBaseConfig,
    SparkBase,
    LimitSwitchConfig,
    ClosedLoopSlot,
    PersistMode,
    ResetMode
)

from constants.constants import IntakeConstants


class IntakeSubsystem(Subsystem):
    def __init__(
        self,
        deployMotorCANID: int,
        deployMotorInverted: bool,
        intakeMotorCANID: int,
        intakeMotorInverted: bool
    ):
        """
        Intake Subsystem.

        This subsystem controls the robot's intake mechanism using two motors and can be
        instantiated multiple times if needed.

        Hardware:
        - Spark MAX: Controls the deploy position of the intake using built-in limit switches.
        - TalonFX (Kraken X60): Drives the intake roller, providing the high torque required
          to pull game pieces into the robot.

        :param deployMotorCANID: CAN ID of the Spark MAX controlling the intake deploy mechanism.
        :param deployMotorInverted: Whether the deploy motor (Spark MAX) is inverted.
        :param intakeMotorCANID: CAN ID of the TalonFX controlling the intake roller.
        :param intakeMotorInverted: Whether the intake roller motor (TalonFX) is inverted.
        """
        super().__init__()

        # Deploy Motor (Spark MAX)
        self.deployMotor = SparkMax(
            deployMotorCANID,
            SparkMax.MotorType.kBrushless
        )

        deployConfig = SparkMaxConfig()

        deployConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        deployConfig.inverted(deployMotorInverted)

        # Enable Spark MAX limit switches
        deployConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        deployConfig.limitSwitch.reverseLimitSwitchEnabled(True)

        deployConfig.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        )

        deployConfig.limitSwitch.reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        )

        # Deploy PID
        deployConfig.closedLoop.pid(
            IntakeConstants.kDeployP,
            IntakeConstants.kDeployI,
            IntakeConstants.kDeployD,
            ClosedLoopSlot.kSlot0
        )

        deployConfig.closedLoop.outputRange(
            IntakeConstants.kDeployMinOutput,
            IntakeConstants.kDeployMaxOutput,
            ClosedLoopSlot.kSlot0
        )

        self.deployMotor.configure(
            deployConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        # Encoder + controller
        self.deployEncoder = self.deployMotor.getEncoder()
        self.deployController = self.deployMotor.getClosedLoopController()

        # Limit switches
        self.forwardLimit = self.deployMotor.getForwardLimitSwitch()
        self.reverseLimit = self.deployMotor.getReverseLimitSwitch()

        # Intake Motor (TalonFX)
        self.intakeMotor = TalonFX(intakeMotorCANID)

        intakeConfig = TalonFXConfiguration()

        intakeConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        intakeConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if intakeMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        self.intakeMotor.configurator.apply(intakeConfig)

        slot0Intake = Slot0Configs()
        (
            slot0Intake
            .with_k_p(IntakeConstants.kIntakeP)
            .with_k_i(IntakeConstants.kIntakeI)
            .with_k_d(IntakeConstants.kIntakeD)
        )

        self.intakeMotor.configurator.apply(slot0Intake)

        self.intakeRequest = VelocityVoltage(0)

        # State
        self._homed = False
        self._isDeployed = False

        self._pulseIntakeActive = False
        self._pulsePositionActive = False
        self._intakePulseTimer = Timer()
        self._positionPulseTimer = Timer()
        self._lastPositionSetpoint = IntakeConstants.kStowPosition

    # Limit helpers
    def forward_limit_pressed(self):
        return self.forwardLimit.get()

    def reverse_limit_pressed(self):
        return self.reverseLimit.get()

    # Periodic
    def periodic(self):

        SmartDashboard.putBoolean("Intake Homed", self._homed)
        SmartDashboard.putBoolean("Intake Deployed", self._isDeployed)
        SmartDashboard.putBoolean("Forward Limit", self.forward_limit_pressed())
        SmartDashboard.putBoolean("Reverse Limit", self.reverse_limit_pressed())

        if not self._homed:

            if self.forward_limit_pressed():

                self.deployEncoder.setPosition(
                    IntakeConstants.kDeployPosition
                )

                self.deployMotor.set(0)

                self._homed = True
                self._isDeployed = True


            elif self.reverse_limit_pressed():

                self.deployEncoder.setPosition(
                    IntakeConstants.kStowPosition
                )

                self.deployMotor.set(0)

                self._homed = True
                self._isDeployed = False


            else:
                self._run_homing()

        else:

            # Intake Roller Pulsing
            if self._pulseIntakeActive:

                if (self._intakePulseTimer.get() % 0.6) < 0.4:
                    velocity = IntakeConstants.kIntakePulseSpeed
                else:
                    velocity = 0

                self.intakeMotor.set_control(
                    self.intakeRequest.with_velocity(velocity)
                )

            # Intake Deploy Pulsing
            if self._pulsePositionActive:

                if (self._positionPulseTimer.get() % 1.0) < 0.5:
                    target = IntakeConstants.kPulsePosition
                else:
                    target = IntakeConstants.kDeployPosition

                if target != self._lastPositionSetpoint:
                    self.deployController.setReference(
                        target,
                        SparkBase.ControlType.kPosition,
                        ClosedLoopSlot.kSlot0
                    )

                    self._lastPositionSetpoint = target

    # Homing
    def _run_homing(self):

        speed = IntakeConstants.kHomeSpeed

        if DriverStation.isAutonomous():
            speed = -speed

        self.deployMotor.set(speed)

    def deploy(self):
        self._pulsePositionActive = False

        if not self._homed:
            return

        self._lastPositionSetpoint = IntakeConstants.kDeployPosition
        self.deployController.setReference(
            IntakeConstants.kDeployPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        )

        self._isDeployed = True


    def stow(self):
        self._pulsePositionActive = False

        if not self._homed:
            return

        self._lastPositionSetpoint = IntakeConstants.kStowPosition
        self.deployController.setReference(
            IntakeConstants.kStowPosition,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        )

        self._isDeployed = False
        
    def toggle_position(self):
        if self._isDeployed:
            self.stow()
        else:
            self.deploy()

    def stop_deploy(self):
        self._pulsePositionActive = False
        self.deployMotor.set(0)

    # Intake Rollers
    def intake(self):
        self._pulseIntakeActive = False

        self.intakeMotor.set_control(
            self.intakeRequest.with_velocity(
                IntakeConstants.kIntakeSpeed
            )
        )


    def intake_reverse(self):
        self._pulseIntakeActive = False

        self.intakeMotor.set_control(
            self.intakeRequest.with_velocity(
                -IntakeConstants.kIntakeSpeed
            )
        )

    def pulse_intake(self):
        if not self._pulseIntakeActive:
            self._pulseIntakeActive = True
            self._intakePulseTimer.restart()

    def pulse_position(self):
        if not self._pulsePositionActive:
            self._pulsePositionActive = True
            self._positionPulseTimer.restart()

    def stop_intake(self):
        self._pulseIntakeActive = False

        self.intakeMotor.set_control(
            self.intakeRequest.with_velocity(0)
        )

    def stop(self):
        self.stop_intake()
        self.stop_deploy()

        self._pulsePositionActive = False
        self._pulseIntakeActive = False

    # State helpers
    def is_homed(self):
        return self._homed

    def is_deployed(self):
        return self._isDeployed