#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license files in the root directory of this project.
#

import commands2

from utils import log

class SetCameraPipeline(commands2.Command):

    def __init__(self, camera, pipelineIndex: int):
        super().__init__()
        self.pipelineIndex = pipelineIndex
        self.camera = camera
        self.addRequirements(camera)

    def initialize(self):
        self.camera.setPipeline(self.pipelineIndex)

    def isFinished(self) -> bool:
        # we are finished when the camera has responded that pipeline index is now set
        if self.camera.getPipeline() == self.pipelineIndex:
            return True
        # otherwise, print that we aren't finished
        log("Vision", "SetCameraPipeline: not yet finished, because camera pipeline = {} and we want {}".format(
            self.camera.getPipeline(), self.pipelineIndex)
        )