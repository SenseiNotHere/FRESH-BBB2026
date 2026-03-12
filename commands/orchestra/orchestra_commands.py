from commands2 import Command
from superstructure import RobotState


class PlaySong(Command):

    def __init__(self, superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):
        self.superstructure.setState(RobotState.PLAYING_SONG)

    def isFinished(self):
        return True

class StopSong(Command):

    def __init__(self, superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):
        self.superstructure.setState(RobotState.IDLE)

    def isFinished(self):
        return True

class ToggleSong(Command):

    def __init__(self, superstructure):
        super().__init__()
        self.superstructure = superstructure

    def initialize(self):

        if self.superstructure.robot_state == RobotState.PLAYING_SONG:
            self.superstructure.setState(RobotState.IDLE)
        else:
            self.superstructure.setState(RobotState.PLAYING_SONG)

    def isFinished(self):
        return True
