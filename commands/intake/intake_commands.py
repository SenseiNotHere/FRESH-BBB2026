from commands2 import Command
from superstructure.superstructure import Superstructure
from superstructure.robot_state import RobotState


class DeployAndRunIntake(Command):

    def __init__(self, superstructure: Superstructure, reverse=False):
        super().__init__()
        self.superstructure = superstructure
        self.reverse = reverse

        if self.superstructure.intake is not None:
            self.addRequirements(superstructure.intake)

    def initialize(self):

        if self.superstructure.intake is None:
            return

        # Deploy if not already deployed
        if not self.superstructure.intake.isDeployed():
            self.superstructure.intake.deploy()

        # Switch to INTAKING state
        if self.reverse:
            self.superstructure.intake.reverse()
        else:
            self.superstructure.intake.startIntaking()
        self.superstructure.setState(RobotState.INTAKING)

    def end(self, interrupted: bool):
        if self.superstructure.intake is not None:
            self.superstructure.intake.stop()
            self.superstructure.intake.stow()
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return False


class ReverseIntake(DeployAndRunIntake):
    def __init__(self, superstructure: Superstructure):
        super().__init__(superstructure, reverse=True)


class RunIntake(DeployAndRunIntake):
    def __init__(self, superstructure: Superstructure):
        super().__init__(superstructure, reverse=False)


class DeployRetractIntake(Command):
    """
    Keeps reversing the intake from deployed to retracted and vice versa, every time you run it
    """
    def __init__(self, superstructure: Superstructure):
        super().__init__()
        self.superstructure = superstructure
        if self.superstructure.intake is not None:
            self.addRequirements(superstructure.intake)

    def initialize(self):
        if self.superstructure.intake is None:
            return

        # Deploy if not already deployed
        if not self.superstructure.intake.isDeployed():
            self.superstructure.intake.deploy()
        else:  # otherwise stow!
            self.superstructure.intake.stow()

    def end(self, interrupted: bool):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return True
