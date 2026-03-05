import math
from dataclasses import dataclass
from typing import Dict

from commands2 import Subsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation
from wpimath.geometry import Rotation2d, Translation3d, Pose2d, Translation2d

from .limelightcamera import LimelightCamera


U_TURN = Rotation2d.fromDegrees(180)
LEARNING_RATE = 0.3
TYPICAL_PERCENT_FRAME = 0.7  # when the tag is ~2m away
EMPHASIZE_TAGS_NEARBY = False

@dataclass
class CameraState:
    camera: LimelightCamera
    cameraPoseOnRobot: Translation3d
    cameraHeadingOnRobot: Rotation2d
    cameraPitchAngleDegrees: float
    minPercentFrame: float
    maxRotationSpeed: float


class LimelightLocalizer(Subsystem):
    def __init__(self, drivetrain, flipIfRed=False):
        super().__init__()

        assert hasattr(drivetrain, "getHeading"), "drivetrain must have getHeading() for localizer to work"
        assert hasattr(drivetrain, "adjustOdometry"), "drivetrain must have adjustOdometry() for localizer to work"
        assert hasattr(drivetrain, "getPose"), "drivetrain must have getPose() for localizer to work"
        self.drivetrain = drivetrain

        self.flipIfRed = flipIfRed

        self.learningRateMult = SendableChooser()
        self.learningRateMult.addOption("300%", 300.0)
        self.learningRateMult.addOption("100%", 1.0)
        self.learningRateMult.addOption("30%", 0.3)
        self.learningRateMult.setDefaultOption("10%", 0.1)
        self.learningRateMult.addOption("3%", 0.03)
        self.learningRateMult.addOption("1%", 0.01)
        SmartDashboard.putData("Localizer/learningRate", self.learningRateMult)

        self.enabled = SendableChooser()
        self.enabled.addOption("off", False)
        self.enabled.setDefaultOption("on", True)
        SmartDashboard.putData("Localizer/on", self.enabled)

        self.allowed = True
        self.cameras: Dict[str, CameraState] = dict()  # list of Limelight cameras

    def addCamera(
        self,
        camera: LimelightCamera,
        cameraPoseOnRobot: Translation3d,
        cameraHeadingOnRobot: Rotation2d,
        cameraPitchAngleDegrees: float = 0.0,
        minPercentFrame: float = 0.07,
        maxRotationSpeed: float = 120,
    ) -> None:
        """
        :param camera: camera to add
        :param cameraPoseOnRobot: is camera x=0.3 meters to the front of the robot center and y=-0.2 meters to right?
        :param cameraHeadingOnRobot: is this camera looking straight forward (Rotation2d.fromDegrees(0)), or maybe right (Rotation2d.fromDegrees(-90)) ?
        :param cameraPitchAngleDegrees: if camera is pitched 10 degrees upwards, set to +10.0, if not pitched then set to 0.0
        :param minPercentFrame: if tags are too small (for example smaller than 0.07% of frame), do not use them
        :param maxRotationSpeed: when robot spins too fast (in degrees per second), camera will be ignored
        """
        assert isinstance(camera, LimelightCamera), "you can only add LimelightCamera(s) to LimelightLocalizer"
        assert camera.cameraName not in self.cameras, f"camera {camera.cameraName} already added to LimelightLocalizer"
        self.cameras[camera.cameraName] = CameraState(
            camera, cameraPoseOnRobot, cameraHeadingOnRobot, cameraPitchAngleDegrees, minPercentFrame, maxRotationSpeed
        )
        camera.addLocalizer()


    def setAllowed(self, value: bool):
        self.allowed = value


    def periodic(self) -> None:
        if len(self.cameras) == 0:
            return

        enabled = self.enabled.getSelected() and self.allowed
        if not enabled:
            return

        flipped = self.flipIfRed and DriverStation.getAlliance() == DriverStation.Alliance.kRed
        learningRate: float = LEARNING_RATE * self.learningRateMult.getSelected()
        odometryPos: Pose2d = self.drivetrain.getPose()
        heading: Rotation2d = self.drivetrain.getHeading()
        rotationSpeed: float = self.drivetrain.getTurnRate()  # rotation speed in degrees per second
        assert heading is not None

        for c in self.cameras.values():
            camera = c.camera
            if not camera.ticked or abs(rotationSpeed) > c.maxRotationSpeed:
                continue

            p = c.cameraPoseOnRobot
            camera.cameraPoseSetRequest.set(
                [p.x, p.y, p.z, c.cameraPitchAngleDegrees, 0.0, c.cameraHeadingOnRobot.degrees()]
            )

            # Limelight4-only (does nothing on Limelight 3)
            camera.imuModeRequest.set(0)
            # 0 - use external imu (the only option available on Limelight 3)
            # 1 - use external imu, seed internal imu
            # 2 - use internal
            # 3 - use internal with MT1 assisted convergence
            # 4 - use internal IMU with external IMU assisted convergence

            if flipped:
                yaw = (heading + U_TURN).degrees()
                camera.robotOrientationSetRequest.set([yaw, 0.0, 0.0, 0.0, 0.0, 0.0])
                botpose = camera.botPoseFlipped.get()
            else:
                yaw = heading.degrees()
                camera.robotOrientationSetRequest.set([yaw, 0.0, 0.0, 0.0, 0.0, 0.0])
                botpose = camera.botPose.get()

            if len(botpose) >= 11:
                # Translation (X,Y,Z), Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image)
                x, y, z, roll, pitch, yaw, latencyMillisec, count, span, distance, percentage = botpose[0:11]
                # SmartDashboard.putNumber("Localizer/" + c.camera.cameraName, percentage)
                if count > 0 and percentage > c.minPercentFrame and not (x == 0 and y == 0):
                    gain = percentage / TYPICAL_PERCENT_FRAME  # tags nearby have more say than tags far away
                    if not EMPHASIZE_TAGS_NEARBY:
                        gain = math.sqrt(gain)
                    shift = Translation2d(x - odometryPos.x, y - odometryPos.y) * min(learningRate * gain, 0.5)
                    self.drivetrain.adjustOdometry(shift, Rotation2d.fromDegrees(0))
