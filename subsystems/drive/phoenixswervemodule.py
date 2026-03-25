import math

from commands2 import Subsystem
from wpilib import Timer, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    CurrentLimitsConfigs,
    Slot0Configs,
)
from phoenix6.signals import (
    NeutralModeValue,
    InvertedValue,
    SensorDirectionValue,
)
from phoenix6.controls import VelocityVoltage, PositionVoltage, VelocityTorqueCurrentFOC, MotionMagicTorqueCurrentFOC
from constants import ModuleConstants


class PhoenixSwerveModuleSubsystem(Subsystem):
    """
    Represents a single swerve module using Phoenix 6.
    Handles drive, steering, encoder fusion, and control.
    """

    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        turnMotorInverted: bool,
        driveMotorInverted: bool,
        canCoderCANId: int,
        canCoderInverted: bool,
        canCoderOffset: float,
        modulePlace: str,
    ) -> None:
        super().__init__()

        # Identity / State
        self.modulePlace = modulePlace
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        # Conversion Factors
        self.driveMotorRotToMeters = (
            ModuleConstants.kWheelCircumferenceMeters
            / ModuleConstants.kDrivingMotorReduction
        )
        self.driveMotorRpsToMps = self.driveMotorRotToMeters

        self.steerMotorRotToRad = (
            2 * math.pi / ModuleConstants.kTurningMotorReduction
        )
        self.radToSteerMotorRot = 1.0 / self.steerMotorRotToRad

        # Encoder Fusion
        self.steerFusedAngle = FusedTurningAngle(modulePlace)
        self.nextFuseTime = 0.0

        # Hardware
        self.drivingMotor = TalonFX(drivingCANId)
        self.turningMotor = TalonFX(turningCANId)
        self.canCoder = CANcoder(canCoderCANId)

        # CANcoder Config
        canCoderConfig = CANcoderConfiguration()
        canCoderConfig.magnet_sensor.magnet_offset = canCoderOffset
        canCoderConfig.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
            if canCoderInverted
            else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.canCoder.configurator.apply(canCoderConfig)

        # Drive Motor Config
        driveConfig = TalonFXConfiguration()
        driveConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        driveConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if driveMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.drivingMotor.configurator.apply(driveConfig)

        driveSlot = Slot0Configs()
        (
            driveSlot
            .with_k_p(ModuleConstants.kDrivingP)
            .with_k_i(ModuleConstants.kDrivingI)
            .with_k_d(ModuleConstants.kDrivingD)
            .with_k_s(ModuleConstants.kDrivingS)
            .with_k_v(ModuleConstants.kDrivingV)
        )
        self.drivingMotor.configurator.apply(driveSlot)

        # Turn Motor Config
        turnConfig = TalonFXConfiguration()
        turnConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        turnConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if turnMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        turnConfig.motion_magic.motion_magic_cruise_velocity = ModuleConstants.kTurningMMCruiseVelocity
        turnConfig.motion_magic.motion_magic_acceleration = ModuleConstants.kTurningMMAcceleration
        turnConfig.motion_magic.motion_magic_jerk = ModuleConstants.kTurningMMJerk
        self.turningMotor.configurator.apply(turnConfig)

        turnSlot = Slot0Configs()
        (
            turnSlot
            .with_k_p(ModuleConstants.kTurningP)
            .with_k_i(ModuleConstants.kTurningI)
            .with_k_d(ModuleConstants.kTurningD)
            .with_k_s(ModuleConstants.kTurningS)
            .with_k_v(ModuleConstants.kTurningV)
            .with_k_a(ModuleConstants.kTurningA)
        )
        self.turningMotor.configurator.apply(turnSlot)

        # Current Limits
        driveLimits = CurrentLimitsConfigs()
        (
            driveLimits
            .with_supply_current_limit(ModuleConstants.kDrivingMotorCurrentLimit)
            .with_stator_current_limit(ModuleConstants.kDrivingMotorStatorCurrentLimit)
            .with_supply_current_limit_enable(True)
            .with_stator_current_limit_enable(True)
        )
        self.drivingMotor.configurator.apply(driveLimits)

        turnLimits = CurrentLimitsConfigs()
        (
            turnLimits
            .with_supply_current_limit(ModuleConstants.kTurningMotorCurrentLimit)
            .with_stator_current_limit(ModuleConstants.kTurningStatorCurrentLimit)
            .with_supply_current_limit_enable(True)
            .with_stator_current_limit_enable(True)
        )
        self.turningMotor.configurator.apply(turnLimits)

        # Control Requests
        velocityVoltageFOC = VelocityTorqueCurrentFOC(0, acceleration=ModuleConstants.kDriveAcceleration)
        self.velocityRequest = velocityVoltageFOC.with_slot(0)
        motionMagicPositonFOC = MotionMagicTorqueCurrentFOC(0)
        self.positionRequest = motionMagicPositonFOC.with_slot(0)

        # Init State
        self.resetEncoders()

    # Periodic / Encoder Fusion

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        if now < self.nextFuseTime:
            return

        self.nextFuseTime = (
            now + ModuleConstants.kFusedAngleRefreshSeconds
        )

        relative = self.turningMotor.get_position()
        absolute = self.canCoder.get_absolute_position()

        if absolute.status.is_ok() and relative.status.is_ok():
            self.steerFusedAngle.observe(
                absolute.value,
                relative.value / ModuleConstants.kTurningMotorReduction,
            )
        elif not absolute.status.is_ok():
            self.steerFusedAngle.complain("absolute encoder unavailable")
        elif not relative.status.is_ok():
            self.steerFusedAngle.complain("relative encoder unavailable")

    # State / Odometry

    def resetEncoders(self) -> None:
        self.drivingMotor.set_position(0)

    def getTurningPosition(self) -> float:
        motorRot = self.steerFusedAngle.to_absolute_rotations(
            self.turningMotor.get_position().value
            / ModuleConstants.kTurningMotorReduction
        )
        return 2 * math.pi * motorRot

    def getState(self) -> SwerveModuleState:
        speed = (
            self.drivingMotor.get_velocity().value
            * self.driveMotorRpsToMps
        )
        return SwerveModuleState(
            speed,
            Rotation2d(self.getTurningPosition()),
        )

    def getPosition(self) -> SwerveModulePosition:
        distance = (
            self.drivingMotor.get_position().value
            * self.driveMotorRotToMeters
        )
        return SwerveModulePosition(
            distance,
            Rotation2d(self.getTurningPosition()),
        )

    # Control

    def setDesiredState(self, desired: SwerveModuleState) -> None:
        if abs(desired.speed) < 0.005:  # m/s, tune 0.02–0.10
            desired = SwerveModuleState(0.0, Rotation2d(self.getTurningPosition()))

        optimized = self._optimizeState(desired)

        driveRps = optimized.speed / self.driveMotorRotToMeters
        self.drivingMotor.set_control(
            self.velocityRequest.with_velocity(driveRps)
        )

        steerRot = self.steerFusedAngle.to_relative_rotations(
            optimized.angle.radians() / (2 * math.pi)
        )
        self.turningMotor.set_control(
            self.positionRequest.with_position(
                steerRot * ModuleConstants.kTurningMotorReduction
            )
        )

        self.desiredState = desired

    def _optimizeState(
        self, desired: SwerveModuleState
    ) -> SwerveModuleState:
        current = Rotation2d(self.getTurningPosition())
        delta = desired.angle.radians() - current.radians()
        flip = False

        while delta > math.pi / 2:
            delta -= math.pi
            flip = not flip
        while delta < -math.pi / 2:
            delta += math.pi
            flip = not flip

        return SwerveModuleState(
            -desired.speed if flip else desired.speed,
            Rotation2d(current.radians() + delta),
        )

    # Telemetry / Extras

    def getTemperature(self):
        """
        :returns: A tuple of (driving, turning) motor temperatures in Celsius.
        """
        return (
            self.drivingMotor.get_device_temp().value,
            self.turningMotor.get_device_temp().value,
        )

    def getSupplyCurrent(self):
        """
        :returns: A tuple of (driving, turning) motor supply currents in amps.
        """
        return (
            self.drivingMotor.get_supply_current(),
            self.turningMotor.get_supply_current(),
        )

    def getMotors(self):
        """
        :yields: The TalonFXs driving and turning the swerve module.
        """
        yield self.drivingMotor
        yield self.turningMotor

# Support Classes

class FusedTurningAngle:
    """
    Fuses relative and absolute encoders to maintain a
    continuous, absolute steering angle.
    """

    def __init__(self, place: str):
        self.place = place
        self.relative_minus_absolute = 0.0
        self.not_ready = "no observations"

    def to_relative_rotations(self, absolute: float) -> float:
        return absolute + self.relative_minus_absolute

    def to_absolute_rotations(self, relative: float) -> float:
        return relative - self.relative_minus_absolute

    def observe(self, absolute: float, relative: float) -> None:
        if self.not_ready:
            self.relative_minus_absolute = relative - absolute
            self.complain("")
            return

        observation = relative - absolute
        while observation - self.relative_minus_absolute > 0.5:
            observation -= 1.0
        while observation - self.relative_minus_absolute < -0.5:
            observation += 1.0

        if ModuleConstants.kTurningKalmanGain > 0:
            correction = (
                ModuleConstants.kTurningKalmanGain
                * (observation - self.relative_minus_absolute)
            )
            self.relative_minus_absolute += correction

    def complain(self, reason: str):
        if reason != self.not_ready:
            SmartDashboard.putString(
                f"Fused Angle {self.place}/not_ready",
                reason,
            )
            self.not_ready = reason
