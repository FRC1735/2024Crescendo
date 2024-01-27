// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem drive = new DriveSubsystem();

  // Controllers
  XboxController driveController = new XboxController(Constants.OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
    configureShuffleboard();

    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drive.drive(
                -MathUtil.applyDeadband(driveController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driveController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driveController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            drive));
  }

  private void configureBindings() {
  }

  private void configureShuffleboard() {
    SmartDashboard.putData("Zero Swerve IMU", new InstantCommand(drive::zeroHeading, drive));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}
