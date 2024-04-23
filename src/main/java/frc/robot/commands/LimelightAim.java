// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightAim extends PIDCommand {
  private static boolean DEBUG = true;

  private final Limelight limelight;
  private final SwerveSubsystem drive;

  /** Creates a new LimelightAim. */
  public LimelightAim(final Limelight limelight, final SwerveSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> limelight.getTx(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
        });

    getController().setTolerance(100);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    addRequirements(limelight, drive);

    this.limelight = limelight;
    this.drive = drive;
  }

  @Override
  public void execute() {
    if (DEBUG) {
      SmartDashboard.putBoolean("at target setpoint", getController().atSetpoint());
      SmartDashboard.putNumber("tx", limelight.getTx());
      SmartDashboard.putNumber("tx error", getController().getPositionError());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return getController().atSetpoint();
    return false;
  }
}
