from commands2 import Command
from superstructure import RobotState


class ToggleIntakePositionCommand(Command):
    """
    Toggles the intake between deployed and stowed positions
    by setting the appropriate robot state on the superstructure.
    """

    def __init__(self, superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):
        if self.superstructure.intake.is_deployed():
            self.superstructure.setState(RobotState.INTAKE_STOWED)
        else:
            self.superstructure.setState(RobotState.INTAKE_DEPLOYED)

    def isFinished(self) -> bool:
        return True