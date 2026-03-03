from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
import commands2
import math

class PoseLockDock(commands2.Command):

    def __init__(self, drivetrain, targetPose: Pose2d):
        super().__init__()
        self.drivetrain = drivetrain
        self.target = targetPose

        # meters -> normalized output
        self.xPID = PIDController(1.0, 0, 0)
        self.yPID = PIDController(1.0, 0, 0)
        self.thetaPID = PIDController(4.0, 0, 0)
        self.thetaPID.enableContinuousInput(-math.pi, math.pi)

        self.maxSpeed = 0.08   # normalized
        self.maxOmega = 0.3    # normalized

        self.addRequirements(drivetrain)

    def execute(self):
        pose = self.drivetrain.getPose()

        xErr = self.target.X() - pose.X()
        yErr = self.target.Y() - pose.Y()
        thetaErr = (self.target.rotation() - pose.rotation()).radians()

        vx = self.xPID.calculate(pose.X(), self.target.X())
        vy = self.yPID.calculate(pose.Y(), self.target.Y())
        omega = self.thetaPID.calculate(pose.rotation().radians(),
                                        self.target.rotation().radians())

        # Clamp HARD
        vx = max(-self.maxSpeed, min(self.maxSpeed, vx))
        vy = max(-self.maxSpeed, min(self.maxSpeed, vy))
        omega = max(-self.maxOmega, min(self.maxOmega, omega))

        self.drivetrain.drive(
            vx, vy, omega,
            fieldRelative=True,
            rateLimit=True,
            square=False
        )

    def isFinished(self):
        pose = self.drivetrain.getPose()

        return (
            abs(self.target.X() - pose.X()) < 0.03 and
            abs(self.target.Y() - pose.Y()) < 0.03 and
            abs((self.target.rotation() - pose.rotation()).radians()) < math.radians(2)
        )

    def end(self, interrupted):
        self.drivetrain.stop()