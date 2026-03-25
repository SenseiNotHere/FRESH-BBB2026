from commands2 import Command
from wpilib import Timer

from subsystems.intake.intakesubsystem import IntakeSubsystem

class RunIntakeRollers(Command):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self):
        self.intake.intake()
    
    def end(self, interrupted: bool):
        self.intake.stop_intake()

    def isFinished(self) -> bool:
        return False

class RunIntakeRollersInverse(Command):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)
    
    def initialize(self):
        self.intake.intake_reverse()
        
    def end(self, interrupted: bool):
        self.intake.stop_intake()
        
    def isFinished(self) -> bool:
        return False

class DeployIntake(Command):

    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self):
        self.intake.deploy()

    def isFinished(self) -> bool:
        return True


class StowIntake(Command):

    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self):
        self.intake.stow()

    def isFinished(self) -> bool:
        return True
    
class PulseIntake(Command):

    def __init__(self, intake: IntakeSubsystem, deploy_at_end: bool):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)
        self.deploy_at_end = deploy_at_end
        self.start_time = 0

    def initialize(self):
        self.start_time = Timer.getFPGATimestamp()

    def execute(self):
        t = Timer.getFPGATimestamp() - self.start_time
        phase = (t % 3.0) > 1.5
        if phase:
            self.intake.intake()
            self.intake.go_to_pulse_position()
        else:
            self.intake.stop_intake()
            self.intake.deploy()

    def end(self, interrupted: bool):
        if self.deploy_at_end:
            self.intake.deploy()
        else:
            self.intake.stow()
        self.intake.stop_intake()

    def isFinished(self) -> bool:
        return False
