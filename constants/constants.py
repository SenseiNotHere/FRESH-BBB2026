import math

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
    kMaxMetersPerSecond = 6.0

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
    kGyroReversed = +1

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
    kDriveAcceleration = 250.0

    kDrivingAutoP = 3.5
    kDrivingAutoI = 0.0
    kDrivingAutoD = 0.0
    kDrivingAutoS = 0.0
    kDrivingAutoV = 0.124

    # Turning PID + FF
    kTurningP = 5.0
    kTurningI = 0.0
    kTurningD = 0.0
    kTurningS = 0.0
    kTurningV = 0.0
    kTurningA = 0.0

    kTurningMMCruiseVelocity = 120.0  # 40 | If still not 150.0
    kTurningMMAcceleration = 400.0  # 120 | If still not 600.0
    kTurningMMJerk = 0.0

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

    kMaxMetersPerSecond = 5.0  # this was meant to be almost as fast as teleop speed limit
    kMaxAccelerationMetersPerSecondSquared = 3.5

    kMaxAngularSpeedRadiansPerSecond = 5.0
    kMaxAngularSpeedRadiansPerSecondSquared = 25.0

    kPController = 3.5
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
    kShooterMotorInverted = True
    kShooterMotor2ID = 10
    kShooterMotor2Inverted = False

    kMinRPM = 600
    kMaxRPM = 4500

    kFF = 0.121
    kP = 0.4
    kI = 0.0001
    kD = 0.0

    kShooterSupplyLimit = 40
    kShooterStatorLimit = 80

    DISTANCE_TO_RPS = InterpolatingMap()
    DISTANCE_TO_RPS.insert(1.0, 48.75) # minimum distance
    DISTANCE_TO_RPS.insert(2.0, 51.75)
    DISTANCE_TO_RPS.insert(2.5, 54.75)
    DISTANCE_TO_RPS.insert(3.0, 56.75)
    DISTANCE_TO_RPS.insert(3.5, 58.0)
    DISTANCE_TO_RPS.insert(4.0, 60.0)
    DISTANCE_TO_RPS.insert(4.5, 63.0)
    DISTANCE_TO_RPS.insert(5.0, 67.5)
    DISTANCE_TO_RPS.insert(5.5, 70.5) # maximum distance

class IntakeConstants:
    # CAN IDs
    kDeployMotorID = 2
    kRollerMotorID = 11

    # Motor Inversions
    kDeployMotorInverted = False
    kRollerMotorInverted = True

    # Deploy (Spark MAX)
    kDeployP = 0.05
    kDeployI = 0.0
    kDeployD = 0.0
    kDeployFF = 0.0

    kDeployMinOutput = -1.0
    kDeployMaxOutput = 1.0

    # Positions (rotations)
    kDeployPosition = 18.0
    kStowPosition = 1.0

    # Homing
    kHomeSpeed = 0.1

    # Anti-jam (position wiggle)
    kPulsePosition = 9.0

    # Rollers (TalonFX)
    
    kIntakeP = 2.6 
    kIntakeI = 0.0
    kIntakeD = 0.0
    kIntakeFF = 0.112

    kIntakeSpeed = 1200
    kIntakePulseSpeed = 30
class IndexerConstants:

    kIndexerMotorID = 1
    kIndexerMotorInverted = True

    kMaxRPS = 40.0
    kFeedRPS = 18.0

    kP = 0.0
    kI = 0.0
    kD = 0.0
    kFF = 0.12

class AgitatorConstants:

    kMotorCANID = 3
    kMotorInverted = True
    kMotor2CANID = 10
    kMotor2Inverted = False
