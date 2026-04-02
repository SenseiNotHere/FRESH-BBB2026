from commands2 import Command
from wpilib import Timer, SmartDashboard

from subsystems.intake.intakesubsystem import IntakeSubsystem

class RunIntakeRollers(Command):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        SmartDashboard.putString("RunIntakeRollers", "created")

    def initialize(self):
        print("RunIntakeRollers initialized")
        SmartDashboard.putString("RunIntakeRollers", "started")
        self.intake.intake()
    
    def end(self, interrupted: bool):
        print("RunIntakeRollers ended")
        self.intake.stop_intake()
        SmartDashboard.putString("RunIntakeRollers", "finished")

    def isFinished(self) -> bool:
        return False

class DoIntake(Command):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.done = False
        self.intake = intake
        self.addRequirements(intake)
        SmartDashboard.putString("DoIntake", "created")

    def execute(self):
        if self.done:
            return
        if self.intake.is_homed():
            SmartDashboard.putString("DoIntake", "deployed")
            self.intake.intake()
            self.intake.deploy()
            self.done = True

    def initialize(self):
        SmartDashboard.putString("DoIntake", "started")
        self.done = False

    def end(self, interrupted: bool):
        self.intake.stop_intake()
        SmartDashboard.putString("DoIntake", "finished")

    def isFinished(self) -> bool:
        return False

class StartIntakeRollers(Command):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)
        
    def initialize(self):
        self.intake.intake()
        
    def isFinished(self) -> bool:
        return True
    
class StopIntakeRollers(Command):
    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)
        
    def initialize(self):
        self.intake.stop_intake()
    
    def isFinished(self) -> bool:
        return True

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
        self.done = False
        SmartDashboard.putString("DeployIntake", "created")

    def initialize(self):
        self.done = False
        SmartDashboard.putString("DeployIntake", "started")

    def execute(self):
        if self.intake.is_homed():
            self.intake.deploy()
            self.done = True

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        SmartDashboard.putString("DeployIntake", "finished")


class StowIntake(Command):

    def __init__(self, intake: IntakeSubsystem):
        super().__init__()
        self.intake = intake
        self.done = False
        self.addRequirements(intake)

    def initialize(self):
        self.done = False

    def execute(self):
        if self.intake.is_homed():
            self.intake.stow()
            self.done = True

    def isFinished(self) -> bool:
        return self.done


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

class AutoPulseAndShoot(Command):
    def __init__(self, intake: IntakeSubsystem, deploy_at_end: bool):
        super().__init__()

        self.intake = intake
        self.addRequirements(intake)
        self.deploy_at_end = deploy_at_end
        self.start_time = 0

    def initialize(self):
        Timer.getFPGATimestamp() - self.start_time

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