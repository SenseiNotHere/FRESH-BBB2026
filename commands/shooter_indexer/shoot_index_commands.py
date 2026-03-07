from commands2 import Command
from wpilib import Timer

from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState


class ShooterConstants:
    TOLERANCE_RPM = 300


class PrepShot(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.ready_since = 0.0
        self.superstructure = superstructure
        if superstructure.shooter is not None:
            self.addRequirements(superstructure.shooter)
        if superstructure.indexer is not None:
            self.addRequirements(superstructure.indexer)
        if superstructure.agitator is not None:
            self.addRequirements(superstructure.agitator)

    def initialize(self):
        self.superstructure.setState(RobotState.PREP_SHOT)

    def execute(self):
        self.setTargetRPS()

        now = Timer.getFPGATimestamp()

        if self.ready_since == 0:
            if self.superstructure.shooter.atSpeed(tolerance_rpm=ShooterConstants.TOLERANCE_RPM):
                self.ready_since = now
        elif now > self.ready_since + 0.12:
            self.feed()

    def setTargetRPS(self):
        if self.superstructure.shotCalculator is not None:
            target_rps = self.superstructure.shotCalculator.getTargetSpeedRPS()
            self.superstructure.shooter.setTargetRPS(target_rps)
        else:
            self.superstructure.shooter.useDashboardPercent()

    def feed(self):
        if self.superstructure.agitator is not None:
            self.superstructure.agitator.feed()
        if self.superstructure.indexer is not None:
            self.superstructure.indexer.feed()

    def end(self, interrupted):
        if self.superstructure.shooter is not None:
            self.superstructure.shooter.stop()
        if self.superstructure.indexer is not None:
            self.superstructure.indexer.stop()
        if self.superstructure.agitator is not None:
            self.superstructure.agitator.stop()

    def isFinished(self):
        return False


class Shoot(PrepShot):

    def initialize(self):
        self.superstructure.setState(RobotState.SHOOTING)

    def execute(self):
        self.setTargetRPS()

        now = Timer.getFPGATimestamp()

        if self.ready_since == 0:
            if self.superstructure.shooter.atSpeed(tolerance_rpm=ShooterConstants.TOLERANCE_RPM):
                self.ready_since = now
        elif now > self.ready_since + 0.0:
            self.feed()


class ManualIndexer(Command):

    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure

        if superstructure.hasIndexer:
            self.addRequirements(superstructure.indexer)

    def initialize(self):
        if self.superstructure.hasIndexer:
            self.superstructure.indexer.feed()

    def end(self, interrupted):
        if self.superstructure.hasIndexer:
            self.superstructure.indexer.stop()

    def isFinished(self):
        return False
