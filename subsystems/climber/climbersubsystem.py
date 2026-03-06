from typing import Optional

from commands2 import Subsystem
from wpilib import DoubleSolenoid, PneumaticsModuleType, SmartDashboard, Timer

from phoenix6.controls import PositionVoltage, VelocityVoltage, DutyCycleOut, MotionMagicVoltage
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    Slot0Configs,
    Slot1Configs,
    CurrentLimitsConfigs
)
from phoenix6.signals import (
    NeutralModeValue,
    InvertedValue,
    FeedbackSensorSourceValue,
    SensorDirectionValue,
    ReverseLimitSourceValue,
    ReverseLimitTypeValue,
    ReverseLimitValue,
    GravityTypeValue
)

from constants.constants import ClimberConstants


class Climber(Subsystem):

    def __init__(
        self,
        motorCANID: int,
        motorInverted: bool,
        canCoderCANID: int,
        canCoderInverted: bool,
        solenoidCANID: int,
        pneumaticsModuleType: PneumaticsModuleType,
        forwardChannel: int,
        reverseChannel: int,
        canCoderOffset: Optional[float] = None
    ):
        super().__init__()

        # Hardware
        self.motor = TalonFX(motorCANID)
        self.motor.clear_sticky_faults()

        self.canCoder = CANcoder(canCoderCANID)

        self.airbrake = DoubleSolenoid(
            module=solenoidCANID,
            moduleType=pneumaticsModuleType,
            forwardChannel=forwardChannel,
            reverseChannel=reverseChannel
        )

        # CANcoder config
        canCoderConfig = CANcoderConfiguration()
        if canCoderOffset is not None:
            canCoderConfig.magnet_sensor.magnet_offset = canCoderOffset

        canCoderConfig.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
            if canCoderInverted
            else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.canCoder.configurator.apply(canCoderConfig)

        # Motor config
        motorConfig = TalonFXConfiguration()
        motorConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motorConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if motorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

        motorConfig.feedback.feedback_sensor_source = (
             FeedbackSensorSourceValue.REMOTE_CANCODER
         )
        motorConfig.feedback.feedback_remote_sensor_id = canCoderCANID

        motorConfig.hardware_limit_switch.reverse_limit_enable = True
        motorConfig.hardware_limit_switch.reverse_limit_source = (
            ReverseLimitSourceValue.LIMIT_SWITCH_PIN
        )
        motorConfig.hardware_limit_switch.reverse_limit_autoset_position_enable = False
        motorConfig.hardware_limit_switch.reverse_limit_autoset_position_value = (
            ClimberConstants.kMinPosition
        )
        motorConfig.hardware_limit_switch.reverse_limit_type = (
            ReverseLimitTypeValue.NORMALLY_OPEN
        )

        self.motor.configurator.apply(motorConfig)

        # PID / Feedforward
        slotConfig = Slot0Configs()
        (
            slotConfig
            .with_k_p(ClimberConstants.kP)
            .with_k_i(ClimberConstants.kI)
            .with_k_d(ClimberConstants.kD)
            .with_k_v(ClimberConstants.kFF)
            .with_k_s(ClimberConstants.kS)
#            .with_k_g(ClimberConstants.kG)
#            .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
        )
        self.motor.configurator.apply(slotConfig)

        slot1Config = Slot1Configs()
        (
            slot1Config
            .with_k_p(ClimberConstants.kVelocityP)
            .with_k_i(ClimberConstants.kI)
            .with_k_d(ClimberConstants.kD)
            .with_k_v(ClimberConstants.kFF)
        )
        self.motor.configurator.apply(slot1Config)

        # Current Limits
        currentLimits = CurrentLimitsConfigs()
        (
            currentLimits
            .with_supply_current_limit(ClimberConstants.kSupplyCurrentLimit)
            .with_supply_current_limit_enable(True)
            .with_stator_current_limit(ClimberConstants.kStatorCurrentLimit)
            .with_stator_current_limit_enable(True)
        )
        self.motor.configurator.apply(currentLimits)
#        self.positionRequest = MotionMagicVoltage(0).with_slot(0).with_enable_foc(True)
        self.positionRequest = PositionVoltage(0).with_slot(0)
        self.velocityRequest = VelocityVoltage(0).with_slot(1)

        # State
        self.targetPosition: float = self.getRelativePosition()
        self.commandedActive: bool = False

        self.jamTimer: float = 0.0
        self.lastTime = Timer.getFPGATimestamp()

        self.airbrakeEngaged = False
        self.airbrake.set(DoubleSolenoid.Value.kReverse)

        # Absolute offsetting + manual/limit-switch zeroing
        self._absolute_position_offset: float = 0.0
        self._prev_reverse_limit: bool = False
        self._in_manual: bool = False

        # Homing
        self._is_homing: bool = False
        self._homed: bool = False
        self._homing_rps: float = ClimberConstants.kManualRPS * -0.5

    # Periodic – Jam Detection

    def periodic(self):
        now = Timer.getFPGATimestamp()
        dt = now - self.lastTime
        self.lastTime = now

        velocity = self.motor.get_velocity().value
        current = self.motor.get_stator_current().value
        position = self.getRelativePosition()

        positionError = abs(self.targetPosition - position)

        tryingToMove = (
            self.commandedActive and
            positionError > ClimberConstants.kPositionDeadband
        )

        moving = abs(velocity) > ClimberConstants.kVelocityDeadband

        if tryingToMove and not moving and current > ClimberConstants.kStallCurrent:
            self.jamTimer += dt
        else:
            self.jamTimer = 0.0

        # Diagnostics
        reverseLimit = (
                self.motor.get_reverse_limit().value
                == ReverseLimitValue.CLOSED_TO_GROUND
        )

        reverseLimitPressed = bool(reverseLimit)
        reverseLimitRisingEdge = reverseLimitPressed and not self._prev_reverse_limit
        self._prev_reverse_limit = reverseLimitPressed

        if reverseLimitRisingEdge:
            # Stop velocity control
            self.motor.set_control(
                self.velocityRequest.with_velocity(0)
            )

            # Zero CANcoder
            self.canCoder.set_position(ClimberConstants.kMinPosition)

            self.targetPosition = ClimberConstants.kMinPosition

            self._is_homing = False
            self._homed = True

            # Hold position using position control
            self.motor.set_control(
                self.positionRequest.with_position(ClimberConstants.kMinPosition)
            )

        atSoftMin = position <= ClimberConstants.kMinPosition + 0.01
        atSoftMax = position >= ClimberConstants.kMaxPosition - 0.01

        SmartDashboard.putNumber("Climber/Position", position)
        SmartDashboard.putNumber("Climber/RotorPosition", self.motor.get_rotor_position().value)
        SmartDashboard.putNumber("Climber/CANCoderPositionRaw", self.canCoder.get_position().value)
        SmartDashboard.putNumber("Climber/CANCoderPosition", self.getAbsolutePosition())
        SmartDashboard.putNumber("Climber/TargetPosition", self.targetPosition)
        SmartDashboard.putNumber("Climber/PositionError", positionError)

        SmartDashboard.putNumber("Climber/Velocity", velocity)
        SmartDashboard.putNumber("Climber/StatorCurrent", current)
        SmartDashboard.putNumber("Climber/SupplyCurrent", self.motor.get_supply_current().value)

        SmartDashboard.putBoolean("Climber/CommandedActive", self.commandedActive)
        SmartDashboard.putBoolean("Climber/InManual", self._in_manual)
        SmartDashboard.putBoolean("Climber/AtTarget", self.atTarget())

        SmartDashboard.putBoolean("Climber/ReverseLimit", reverseLimitPressed)
        SmartDashboard.putBoolean("Climber/AtSoftMin", atSoftMin)
        SmartDashboard.putBoolean("Climber/AtSoftMax", atSoftMax)

        SmartDashboard.putBoolean("Climber/AirbrakeEngaged", self.airbrakeEngaged)
        SmartDashboard.putNumber("Climber/JamTimer", self.jamTimer)

        SmartDashboard.putBoolean("Climber/Homed", self._homed)

        # Jam Status
        if self.jamTimer > ClimberConstants.kStallTime:
            self.stop()
            SmartDashboard.putBoolean("Climber/Jammed", True)
        else:
            SmartDashboard.putBoolean("Climber/Jammed", False)

        # Why Not Moving Debug
        reason = "OK"

        if self.airbrakeEngaged:
            reason = "Airbrake Engaged"
        elif reverseLimitPressed:
            reason = "Reverse Limit Switch"
        elif atSoftMin:
            reason = "Soft Min Limit"
        elif atSoftMax:
            reason = "Soft Max Limit"
        elif not self.commandedActive:
            reason = "Not Commanded"
        elif abs(velocity) < ClimberConstants.kVelocityDeadband:
            reason = "Velocity Deadband"

        SmartDashboard.putString("Climber/WhyNotMoving", reason)

        SmartDashboard.putBoolean("Climber/Airbrake", self.airbrakeEngaged)

    # Position Control

    def setPosition(self, pos: float):

        if self.airbrakeEngaged:
            return  # Do not allow movement while brake engaged

        self._in_manual = False

        pos = max(
            ClimberConstants.kMinPosition,
            min(pos, ClimberConstants.kMaxPosition)
        )

        self.targetPosition = pos
        self.commandedActive = True

        self.motor.set_control(
            self.positionRequest.with_position(pos)
        )

    def stop(self):
        self.commandedActive = False
        self._in_manual = False
        self.motor.set_control(
            self.positionRequest.with_position(
                self.getRelativePosition()
            )
        )

    def startHoming(self):

        if self._homed:
            return

        # If already at limit, just zero immediately
        reverseLimit = (
                self.motor.get_reverse_limit().value
                == ReverseLimitValue.CLOSED_TO_GROUND
        )

        if reverseLimit:
            self.canCoder.set_position(ClimberConstants.kMinPosition)
            self.targetPosition = ClimberConstants.kMinPosition
            self._homed = True
            return

        self.releaseAirbrake()

        self._is_homing = True
        self.commandedActive = True

        self.motor.set_control(
            self.velocityRequest.with_velocity(self._homing_rps)
        )

    def atTarget(self) -> bool:
        position_error = abs(
            self.targetPosition - self.getRelativePosition()
        )

        return (
            position_error < ClimberConstants.kPositionDeadband
        )

    def atHighTarget(self) -> bool:
        """
        Risen height
        """
        position_error = abs(
            self.targetPosition - self.getRelativePosition()
        )

        return (position_error < ClimberConstants.kPositionDeadband) and (self.targetPosition == ClimberConstants.kRisenHeight)

    def atLowTarget(self) -> bool:
        """
        Min height
        """
        position_error = abs(
            self.targetPosition - self.getRelativePosition()
        )

        return (position_error < ClimberConstants.kPositionDeadband) and (self.targetPosition == ClimberConstants.kMinPosition)

    def atClimbTarget(self):
        """
        Climbed height
        """
        position_error = abs(
            self.targetPosition - self.getRelativePosition()
        )

        return (position_error < ClimberConstants.kPositionDeadband) and (self.targetPosition == ClimberConstants.kClimbedHeight)

    # Manual Adjust

    def manualVelocity(self, joystickValue: float):

        deadband = 0.1

        if abs(joystickValue) < deadband:
            self.stop()
            return

        self._in_manual = True

        # Release brake automatically in manual
        if self.airbrakeEngaged:
            self.releaseAirbrake()

        # Closed-loop velocity control
        target_rps = joystickValue * ClimberConstants.kManualRPS

        self.commandedActive = True
        self.motor.set_control(
            self.velocityRequest.with_velocity(target_rps)
        )

    # Airbrake

    def engageAirbrake(self):
        self.airbrake.set(DoubleSolenoid.Value.kForward)
        self.airbrakeEngaged = True

    def releaseAirbrake(self):
        self.airbrake.set(DoubleSolenoid.Value.kReverse)
        self.airbrakeEngaged = False

    # Sensors

    def getRelativePosition(self) -> float:
        return self.motor.get_position().value

    def getAbsolutePosition(self) -> float:
        return self.canCoder.get_position().value + self._absolute_position_offset

    # Motors
    def getMotors(self):
        yield self.motor