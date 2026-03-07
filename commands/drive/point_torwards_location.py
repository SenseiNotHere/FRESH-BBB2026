#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
from typing import Callable

import commands2

from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Translation2d

from subsystems.drive.drivesubsystem import DriveSubsystem


class PointTowardsLocation(commands2.Command):
    """
    One can use this command to have their swerve robot keep pointing towards some location as it moves around.
    Example (this can go into robotcontaier.py, inside of configureButtonBindings() function):

        ```
            from commands.point_towards_location import PointTowardsLocation

            # create a command for keeping the robot nose pointed towards the hub
            keepPointingTowardsHub = PointTowardsLocation(
                drivetrain=self.robotDrive,
                location=Translation2d(x=4.59, y=4.025),
                locationIfRed=Translation2d(x=11.88, y=4.025),
            )

            # setup a condition for when to do this: do it when the joystick right trigger is pressed by more than 50%
            whenRightTriggerPressed = self.driverController.axisGreaterThan(
                XboxController.Axis.kRightTrigger, threshold=0.5
            )

            # connect the command to its trigger
            whenRightTriggerPressed.whileTrue(keepPointingTowardsHub)

        ```
    """

    def __init__(
            self,
            drivetrain: DriveSubsystem,
            location: Translation2d | Callable[[], Translation2d | None],
            locationIfRed: Translation2d | Callable[[], Translation2d | None]
    ):
        super().__init__()
        self.location, self.locationIfRed = location, locationIfRed
        self.drivetrain = drivetrain  # not calling addRequirement, on purpose

        self.activeTargetLocation: Translation2d | None = None
        self.active = False


    def initialize(self):
        self.active = False
        color = self.drivetrain.getAlliance()
        if color == DriverStation.Alliance.kRed:
            self.activeTargetLocation = self.locationIfRed
            SmartDashboard.putString("command/c" + self.__class__.__name__, "assuming red alliance")
        else:
            self.activeTargetLocation = self.location
            SmartDashboard.putString("command/c" + self.__class__.__name__, "assuming blue or unknown alliance")

        raw = (
            self.locationIfRed
            if self.drivetrain.getAlliance() == DriverStation.Alliance.kRed
            else self.location
        )

        self.activeTargetLocation = self._resolve(raw)

        if self.activeTargetLocation is None:
            return

    def execute(self):
        # heading override already in place?
        if self.active:
            return

        # Ensure activeTargetLocation is not None
        if self.activeTargetLocation is None:
            SmartDashboard.putString(
                "command/c" + self.__class__.__name__,
                "Error: activeTargetLocation is None"
            )
            return

        # try to place that heading override now
        if self.drivetrain.startOverrideToFaceThisPoint(self.activeTargetLocation):
            self.active = True
            SmartDashboard.putString(
                "command/c" + self.__class__.__name__,
                f"pointing to x, y: {self.activeTargetLocation.x}, {self.activeTargetLocation.y}")


    def end(self, interrupted: bool):
        self.drivetrain.stop()

    def isFinished(self) -> bool:
        return False  # never finish, wait for user to stop this command

    def _resolve(self, value):
        # Log the input value to _resolve
        SmartDashboard.putString(
            "command/c" + self.__class__.__name__ + "/_resolve_input",
            f"Input value: {value}"
        )

        resolved_value = value() if callable(value) else value

        # Log the resolved value
        SmartDashboard.putString(
            "command/c" + self.__class__.__name__ + "/_resolve_output",
            f"Resolved value: {resolved_value}"
        )

        return resolved_value
