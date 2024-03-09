// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Axel;

public class RotateAxel extends Command {
  /** Creates a new RotateAxel. */
  private Axel axel;
  private double targetangle;
  public RotateAxel(Axel axel, double targetangle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(axel);
    this.axel = axel;
    this.targetangle = targetangle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    axel.setReference(targetangle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.axel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return axel.isAtTarget();
  }
}
