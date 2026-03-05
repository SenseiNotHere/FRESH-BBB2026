from __future__ import annotations

import math
import typing

import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)
from wpilib import (
    SmartDashboard,
    Field2d,
    DriverStation,
    Timer
)

from commands.drive.aim_to_direction import AimToDirectionConstants
from .phoenixswervemodule import PhoenixSwerveModule

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants

from constants.constants import (
    DrivingConstants,
    ModuleConstants,
    AutoConstants
)
from commands.drive.holonomic_drive import HolonomicDrive
from .wrapped_navx import NavxGyro

U_TURN = Rotation2d.fromDegrees(180)


class DriveSubsystem(Subsystem):
    def __init__(self, maxSpeedScaleFactor=None) -> None:
        super().__init__()

        if maxSpeedScaleFactor is not None:
            assert callable(maxSpeedScaleFactor)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        self.frontLeft = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kFrontLeftDriving,
            turningCANId=DrivingConstants.kFrontLeftTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kFrontLeftDriveMotorInverted,
            canCoderCANId=DrivingConstants.kFrontLeftTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kFrontLeftTurningEncoderOffset,
            modulePlace="FL"
        )

        self.frontRight = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kFrontRightDriving,
            turningCANId=DrivingConstants.kFrontRightTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kFrontRightDriveMotorInverted,
            canCoderCANId=DrivingConstants.kFrontRightTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kFrontRightTurningEncoderOffset,
            modulePlace="FR"
        )

        self.backLeft = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kBackLeftDriving,
            turningCANId=DrivingConstants.kBackLeftTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kBackLeftDriveMotorInverted,
            canCoderCANId=DrivingConstants.kBackLeftTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kBackLeftTurningEncoderOffset,
            modulePlace="BL"
        )

        self.backRight = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kBackRightDriving,
            turningCANId=DrivingConstants.kBackRightTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kBackRightDriveMotorInverted,
            canCoderCANId=DrivingConstants.kBackRightTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kBackRightTurningEncoderOffset,
            modulePlace="BR"
        )

        # Override for the direction where robot should point
        self.overrideControlsToFaceThisPoint: Translation2d | None = None

        # Navx SPI gyro
        self.gyro = NavxGyro()

        # another possibility:
        # from phoenix6.hardware import Pigeon2
        # self.gyro = Pigeon2(0)  # 0 is its CAN ID

        self.xySpeedLimiter = SlewRateLimiter2d(DrivingConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DrivingConstants.kRotationalSlewRate)

        #Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DrivingConstants.kDriveKinematics,
            Rotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        self.odometryHeadingOffset = Rotation2d(0)
        self.resetOdometry(Pose2d(14.0, 4.05, Rotation2d.fromDegrees(180)))

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        self.simPhysics = None

        AutoBuilder.configure(
            self.getPose,
            self.resetOdometryAuto,
            self.getRobotRelativeSpeeds,
            lambda speeds, _: self.driveRobotRelativeChassisSpeeds(ChassisSpeeds(speeds.vx, speeds.vy, -speeds.omega)),
            PPHolonomicDriveController(
                PIDConstants(AutoConstants.kPController, 0, 0),
                PIDConstants(AutoConstants.kPThetaController, 0, 0)
            ),
            AutoConstants.config,
            self.shouldFlipPath,
            self
        )

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """Returns the current robot-relative ChassisSpeeds"""
        return DrivingConstants.kDriveKinematics.toChassisSpeeds(
            (
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState(),
            )
        )

    @staticmethod
    def shouldFlipPath():
        """
        :return: Whether to flip the path based on alliance color
        """
        return False #DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self) -> None:
        if self.simPhysics is not None:
            self.simPhysics.periodic()

        # Sync turning encoders on all modules to prevent drift
        self.frontLeft.periodic()
        self.frontRight.periodic()
        self.backLeft.periodic()
        self.backRight.periodic()

        # Update the odometry in the periodic block
        pose = self.odometry.update(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.field.setRobotPose(pose)

        # Data Puts
        # Pose
        SmartDashboard.putNumber("X Coordinate", pose.x)
        SmartDashboard.putNumber("Y Coordinate", pose.y)
        SmartDashboard.putNumber("Heading (deg)", pose.rotation().degrees())

        # Temperatures
        SmartDashboard.putNumberArray("Front Left Temp", self.frontLeft.getTemperature())
        SmartDashboard.putNumberArray("Front Right Temp", self.frontRight.getTemperature())
        SmartDashboard.putNumberArray("Back Left Temp", self.backLeft.getTemperature())
        SmartDashboard.putNumberArray("Back Right Temp", self.backRight.getTemperature())

        # Positions
        SmartDashboard.putNumber("Front Left Position", self.frontLeft.getPosition().angle.degrees())
        SmartDashboard.putNumber("Front Right Position", self.frontRight.getPosition().angle.degrees())
        SmartDashboard.putNumber("Back Left Position", self.backLeft.getPosition().angle.degrees())
        SmartDashboard.putNumber("Back Right Position", self.backRight.getPosition().angle.degrees())

    def getHeading(self) -> Rotation2d:
        """
        :return: The robot's heading as a Rotation2d
        """

        return self.getPose().rotation()

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d, resetGyro=True) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.
        :param resetGyro: Whether to reset the gyro heading to the pose heading.
        """
        if resetGyro:
            self.gyro.reset()

        self.odometry.resetPosition(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )
        self.odometryHeadingOffset = self.odometry.getPose().rotation() - self.getGyroHeading()

    def resetOdometryAuto(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )

    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        """Adjusts the odometry by a specified translation and rotation delta.
        :param dTrans: The translation delta to apply.
        :param dRot: The rotation delta to apply.
        """
        pose = self.getPose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)
        self.odometry.resetPosition(
            pose.rotation() - self.odometryHeadingOffset,
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            newPose,
        )
        self.odometryHeadingOffset += dRot

    def stop(self):
        """
        Stops the robot by setting all speeds to zero.
        """
        self.arcadeDrive(0, 0)

    def arcadeDrive(
            self,
            xSpeed: float,
            rot: float,
            assumeManualInput: bool = False,
    ) -> None:
        """
        Drive the robot using arcade controls.
        :param xSpeed: forward speed
        :param rot: rotation speed
        :param assumeManualInput: whether to square the inputs for manual control
        """
        self.drive(xSpeed, 0, rot, False, False, square=assumeManualInput)

    def rotate(self, rotSpeed) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)
        :param rotSpeed: rotation speed
        """
        self.arcadeDrive(0, rotSpeed)

    def drive(
            self,
            xSpeed: float,
            ySpeed: float,
            rot: float,
            fieldRelative: bool,
            rateLimit: bool,
            square: bool = False
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        :param square:        Whether to square the inputs (useful for manual control)
        """

        # Apply constraints and transformations to the user (joystick) input
        if square:
            rot = rot * abs(rot)
            norm = math.hypot(xSpeed, ySpeed)
            xSpeed = xSpeed * norm
            ySpeed = ySpeed * norm
        if (xSpeed != 0 or ySpeed != 0) and self.maxSpeedScaleFactor is not None:
            norm = math.hypot(xSpeed, ySpeed)
            scale = abs(self.maxSpeedScaleFactor() / norm)
            if scale < 1:
                xSpeed = xSpeed * scale
                ySpeed = ySpeed * scale
        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        if self.overrideControlsToFaceThisPoint:
            rot = self.calaculateOverrideRotSpeed()

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedGoal = xSpeedCommanded * DrivingConstants.kMaxMetersPerSecond
        ySpeedGoal = ySpeedCommanded * DrivingConstants.kMaxMetersPerSecond
        rotSpeedGoal = rot * DrivingConstants.kMaxAngularSpeed

        # field relative conversion must happen before rate limiting, since rate limiting is optional
        if fieldRelative:
            heading = self.getPose().rotation()
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                heading = heading + U_TURN
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedGoal, ySpeedGoal, rotSpeedGoal, heading)
        else:
            targetChassisSpeeds = ChassisSpeeds(xSpeedGoal, ySpeedGoal, rotSpeedGoal)

        # rate limiting has to be applied this way, to keep the rate limiters current (with time)
        slewedX, slewedY = self.xySpeedLimiter.calculate(targetChassisSpeeds.vx, targetChassisSpeeds.vy)
        slewedRot = self.rotLimiter.calculate(targetChassisSpeeds.omega)
        if rateLimit:
            targetChassisSpeeds.vx, targetChassisSpeeds.vy, targetChassisSpeeds.omega = slewedX, slewedY, slewedRot

        # from chassis speed targets, calculate and set the wheel speeds targets
        swerveModuleStates = DrivingConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds)
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DrivingConstants.kMaxMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def driveRobotRelativeChassisSpeeds(self, speeds: ChassisSpeeds, feedforwards=None) -> None:
        states = DrivingConstants.kDriveKinematics.toSwerveModuleStates(speeds)

        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states, DrivingConstants.kMaxMetersPerSecond
        )

        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.backLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.backRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(
            self,
            desiredStates: typing.Tuple[
                SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
            ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DrivingConstants.kMaxMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.backLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backRight.resetEncoders()

    def isSteerReady(self, tolerance_rad=2.0/57.) -> bool:
        """
        Returns True if all swerve modules are within tolerance of their desired angle.
        :param tolerance_rad: Allowed angle error (degrees) before a module is considered aligned.
        :return:
        """
        modules = [
            self.frontLeft,
            self.frontRight,
            self.backLeft,
            self.backRight,
        ]

        for m in modules:
            error = abs(
                m.getTurningPosition()
                - m.desiredState.angle.radians()
            )
            if math.degrees(error) > tolerance_rad:
                return False

        return True

    def getGyroHeading(self) -> Rotation2d:
        """Returns the heading of the robot, tries to be smart when gyro is disconnected

        :returns: the robot's heading as Rotation2d
        """
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value * DrivingConstants.kGyroReversed)

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.get_angular_velocity_z_device().value * DrivingConstants.kGyroReversed

    def getTurnRateDegreesPerSec(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.getTurnRate() * 180 / math.pi

    def getMotors(self):
        """
        :yields: The motors controlled by the swerve modules.
        """
        for module in (
            self.frontLeft,
            self.frontRight,
            self.backLeft,
            self.backRight,
        ):
            yield from module.getMotors()


    def startOverrideToFaceThisPoint(self, point: Translation2d) -> bool:
        if self.overrideControlsToFaceThisPoint is not None:
            return False
        self.overrideControlsToFaceThisPoint = point
        return True


    def stopOverrideToFaceThisPoint(self, point: Translation2d):
        if self.overrideControlsToFaceThisPoint == point:
            self.overrideControlsToFaceThisPoint = None
            return True
        return False


    def calaculateOverrideRotSpeed(self):
        # 1. how many degrees we need to turn?
        pose = self.getPose()
        vectorToTarget = self.overrideControlsToFaceThisPoint - pose.translation()
        if not vectorToTarget.squaredNorm() > 0:
            return 0.0
        targetDirection = vectorToTarget.angle()
        degreesRemainingToTurn = (targetDirection - pose.rotation()).degrees()

        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemainingToTurn > 180:
            degreesRemainingToTurn -= 360
        while degreesRemainingToTurn < -180:
            degreesRemainingToTurn += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemainingToTurn)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        rotSpeed = min(proportionalSpeed, 1.0)

        # 3. if need to turn left, return the positive speed, otherwise negative
        return rotSpeed if degreesRemainingToTurn > 0 else -rotSpeed

class SlewRateLimiter2d:
    """
    Slew rate limiter for a near-square drivetrain: limits the total acceleration along X axis and Y axis
    (better than total sqrt(x^2 + y^2) acceleration because the drivetrain is not a circle)
    """

    def __init__(self, rate) -> None:
        self.rate = rate
        self.t = Timer.getFPGATimestamp()
        self.x = self.y = 0.0

    def calculate(self, x, y) -> typing.Tuple[float, float]:
        t = Timer.getFPGATimestamp()

        dx = x - self.x
        dy = y - self.y

        # this is the maximum permitted change in X or Y, given the slew rate
        limit = self.rate * abs(t - self.t)
        self.t = t

        # this is the desired change in X or Y
        change = max(abs(dx), abs(dy))

        # is this change above the limit? if yes, apply slew rate
        if limit > 0:
            if change < limit:
                self.x, self.y = x, y
            else:
                fraction = limit / change
                self.x += dx * fraction
                self.y += dy * fraction

        return self.x, self.y


class BadSimPhysics(object):
    """
    this is the wrong way to do it, it does not scale!!!
    the right way is shown here: https://github.com/robotpy/examples/blob/main/Physics/src/physics.py
    and documented here: https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
    (but for a swerve drive it will take some work to add correctly)
    """
    def __init__(self, drivetrain: DriveSubsystem, robot: wpilib.RobotBase):
        self.drivetrain = drivetrain
        self.robot = robot
        self.t = 0

    def periodic(self):
        past = self.t
        self.t = wpilib.Timer.getFPGATimestamp()
        if past == 0:
            return  # it was first time

        dt = self.t - past
        if self.robot.isEnabled():
            drivetrain = self.drivetrain

            states = (
                drivetrain.frontLeft.desiredState,
                drivetrain.frontRight.desiredState,
                drivetrain.backLeft.desiredState,
                drivetrain.backRight.desiredState,
            )
            speeds = DrivingConstants.kDriveKinematics.toChassisSpeeds(states)

            dx = speeds.vx * dt
            dy = speeds.vy * dt

            heading = drivetrain.getHeading()
            trans = Translation2d(dx, dy).rotateBy(heading)
            rot = (speeds.omega * 180 / math.pi) * dt

            g = drivetrain.gyro
            g.set_yaw(g.get_yaw().value + rot * DrivingConstants.kGyroReversed)
            drivetrain.adjustOdometry(trans, Rotation2d())
