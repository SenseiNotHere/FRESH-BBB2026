#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license files in the root directory of this project.
#

import typing
import wpilib
from commands2 import CommandScheduler, TimedCommandRobot, Command

from robotcontainer import RobotContainer


class MyRobot(TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[Command] = None
    testCommand: typing.Optional[Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.robotContainer = RobotContainer(self)

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""


    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        self.robotContainer.updateAutoPreview()

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.robotContainer.getAutonomousCommand()

        # schedule the autonomous command (example)
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        self.robotContainer.autonomousSubsystem.clearAutoPreview()


    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def robotPeriodic(self) -> None:
        super().robotPeriodic()  # runs scheduler
        self.robotContainer.superstructure.update()

    def testInit(self) -> None:
        # Cancels all running subsystems at the start of test mode
        CommandScheduler.getInstance().cancelAll()
        self.testCommand = self.robotContainer.getTestCommand()

        # schedule the autonomous command (example)
        if self.testCommand is not None:
            self.testCommand.schedule()

    def testPeriodic(self) -> None:
        """This function is called periodically during test mode"""