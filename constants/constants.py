import math

from wpilib import PneumaticsModuleType
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from phoenix6.signals import NeutralModeValue

from pathplannerlib.config import RobotConfig, ModuleConfig, DCMotor

from utils.interpolatingMap import InterpolatingMap


class KrakenX60:
    kFreeSpeedRpm = 5800
    kMaxSpeedMetersPerSecond = 2.8

class DrivingConstants:

    # Physical Limits
    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    # Slew Rate Limiting
    kMagnitudeSlewRate = 9.8
    kRotationalSlewRate = 12.0

    # Robot Geometry
    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kModulePositions = [
        Translation2d(+kWheelBase / 2, +kTrackWidth / 2),
        Translation2d(+kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, +kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]

    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # CAN IDs – Drive Motors
    kFrontLeftDriving = 7
    kFrontRightDriving = 5
    kBackLeftDriving = 1
    kBackRightDriving = 3

    # CAN IDs – Turning Motors
    kFrontLeftTurning = 8
    kFrontRightTurning = 6
    kBackLeftTurning = 2
    kBackRightTurning = 4

    # CAN IDs – Absolute Encoders
    kFrontLeftTurningEncoder = 4
    kFrontRightTurningEncoder = 3
    kBackLeftTurningEncoder = 1
    kBackRightTurningEncoder = 2

    # Gyro
    kGyroReversed = -1

    # Lock Deadbands
    kLockVxDeadband = 0.05
    kLockVyDeadband = 0.05
    kLockOmegaDeadband = 0.10

class ModuleConstants:

    # Gear Ratios / Mechanics
    kDrivingMotorPinionTeeth = 14
    kDrivingMotorReduction = 6.12
    kTurningMotorReduction = 287 / 11.0

    kWheelDiameterMeters = ((0.0965 / 0.97) / 0.98)
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    # Derived Values
    kDrivingMotorFreeSpeedRps = KrakenX60.kFreeSpeedRpm / 60.0
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    # Motor Inversions
    kTurningEncoderInverted = False
    kTurningMotorInverted = False

    kFrontLeftDriveMotorInverted = True
    kFrontRightDriveMotorInverted = False
    kBackLeftDriveMotorInverted = True
    kBackRightDriveMotorInverted = False

    # Absolute Encoder Offsets
    kFrontLeftTurningEncoderOffset = 0.264892578125
    kFrontRightTurningEncoderOffset = 0.22265625
    kBackLeftTurningEncoderOffset = -0.080078125
    kBackRightTurningEncoderOffset = 0.09521484375

    # Turning Encoder Conversion
    kTurningEncoderPositionFactor = math.tau
    kTurningEncoderVelocityFactor = math.tau / 60.0
    kTurningEncoderPositionPIDMinInput = 0.0
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

    # Drive PID + FF
    kDrivingP = 0.3
    kDrivingI = 0.0
    kDrivingD = 0.0
    kDrivingS = 0.0
    kDrivingV = 0.124
    kMagicMotionAcceleration = 250.0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps

    kDrivingMinOutput = -1.0
    kDrivingMaxOutput = 1.0

    # Turning PID + FF
    kTurningP = 10.0
    kTurningI = 0.0
    kTurningD = 0.1
    kTurningS = 0.1
    kTurningV = 2.49
    kTurningA = 0.0

    kTurningDeadbandRot = 0.002
    kTurningMinOutput = -1.0
    kTurningMaxOutput = 1.0

    # Drift / Sync
    kFusedAngleRefreshSeconds = 0.02
    kTurningKalmanGain = 0.05
    kTurningDriftDegrees = 10.0
    kTurningSyncMaxVelocity = 0.5
    kDrivingSyncMaxVelocity = 0.2

    # Neutral Modes
    kDrivingMotorIdleMode = NeutralModeValue(NeutralModeValue.BRAKE)
    kTurningMotorIdleMode = NeutralModeValue(NeutralModeValue.COAST)

    # Current Limits
    kDrivingMotorCurrentLimit = 70
    kDrivingMotorStatorCurrentLimit = 120
    kTurningMotorCurrentLimit = 40
    kTurningStatorCurrentLimit = 60

    # Misc
    kDrivingMinSpeedMetersPerSecond = 0.01
    kSteerDriveCouplingRatio = 3.857142857142857
    kSteerKs = 0.1
    kSteerHoldDeadband = math.radians(0.25) * (
        1.0 / ((2 * math.pi) / kTurningMotorReduction)
    )

class OIConstants:
    # Driver controller
    kDriverControllerPort = 0
    kDriveDeadband = 0.05

    # Operator controller
    kOperatorControllerPort = 1

class AutoConstants:

#    moduleConfig = ModuleConfig(
#        maxDriveVelocityMPS=DrivingConstants.kMaxMetersPerSecond,
#        driveMotor=DCMotor.krakenX60().withReduction(ModuleConstants.kDrivingMotorReduction),
#        driveCurrentLimit=ModuleConstants.kDrivingMotorCurrentLimit,
#        numMotors=4,
#        wheelRadiusMeters=ModuleConstants.kWheelDiameterMeters / 2,
#        wheelCOF=1.0
#    )

#    config = RobotConfig(
#        massKG=68,
#        MOI=8.0,
#        moduleConfig=moduleConfig,
#        moduleOffsets=DrivingConstants.kModulePositions
#    )

    config = RobotConfig.fromGUISettings()

    kUseSqrtControl = True

    kMaxMetersPerSecond = 1.2
    kMaxAccelerationMetersPerSecondSquared = 3.5

    kMaxAngularSpeedRadiansPerSecond = 5.0
    kMaxAngularSpeedRadiansPerSecondSquared = 25.0

    kPController = 1.0
    kPThetaController = 0.5

    kIXController = 0.0
    kIThetaController = 0.0

    kDXController = 0.0
    kDThetaController = 0.0

    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )

class ShooterConstants:

    kShooterMotorID = 9

    kMinRPM = 600
    kMaxRPM = 4500

    kFF = 0.121
    kP = 0.4
    kI = 0.0001
    kD = 0.0

    kShooterSupplyLimit = 40
    kShooterStatorLimit = 140

    DISTANCE_TO_RPS = InterpolatingMap()
    DISTANCE_TO_RPS.insert(2.57, 3375 / 60)
    DISTANCE_TO_RPS.insert(3.0, 3465 / 60)
    DISTANCE_TO_RPS.insert(3.2, 3510 / 60)
    DISTANCE_TO_RPS.insert(3.35, 60)
    DISTANCE_TO_RPS.insert(3.40, 3555 / 60)
    DISTANCE_TO_RPS.insert(3.6, 3622.5 / 60)
    DISTANCE_TO_RPS.insert(3.8, 3690 / 60)
    DISTANCE_TO_RPS.insert(4.0, 3780  / 60)
    DISTANCE_TO_RPS.insert(4.2, 3825 / 60)
    DISTANCE_TO_RPS.insert(4.49, 63.75)


class PneumaticsConstants:
    kPCMID = 0
    kModuleType = PneumaticsModuleType.CTREPCM

class IntakeConstants:

    kIntakeMotorCANID = 1

    kIntakeRPS = 20.0

    kFF = 0.0
    kP = 0.0
    kD = 0.0

    kPneumaticsModuleType = PneumaticsModuleType.CTREPCM
    kSolenoidModuleID = 0
    kSolenoidForwardChannel = 1
    kSolenoidReverseChannel = 0

class IndexerConstants:

    kIndexerMotorID = 2

    kMaxRPS = 40.0
    kFeedRPS = 18.0

    kP = 0.0
    kI = 0.0
    kD = 0.0
    kFF = 0.12

class AgitatorConstants:

    kMotorCANID = 3
    kMotorInverted = False

class ClimberConstants:

    kMotorID = 10
    kMotorInverted = False
    kCanCoderCANID = 5

    kPneumaticsModuleType = PneumaticsModuleType.CTREPCM
    kPCMID = 0
    kForwardChannel = 2
    kReverseChannel = 3

    kMaxPosition = -0.002
    kMinPosition = -1.999
    kRisenHeight = -0.002
    kClimbedHeight = -1.5
    kHeightTolerance = 0.03
    kPositionDeadband = 0.06
    kVelocityDeadband = 0.0
    kStallCurrent = 55.0
    kStallTime = 0.3
    kManualSpeed = 1.0  # rotations per second

    kP = 20.0
    kI = 0.0
    kD = 0.0
    kS = 2.4
    kFF = 0.0
    kG = 0.5 # Normal values 0.5->2.0

    kVelocityP = 2.0

    kStatorCurrentLimit = 60
    kSupplyCurrentLimit = 40

    kManualRPS = 3.0

class TrenchAssistConstants:

    kTrenchAssistEnabled = False
    kTriggerDistance = 0.9
