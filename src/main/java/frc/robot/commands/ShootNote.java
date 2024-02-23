// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  private Shooter shooter;
  private Collector collector;
  private long startTime;
  private long SHOOTER_WARM_UP_TIME_MILLIS = 2000; // TODO - be smarter about this

  /** Creates a new ShootNote. */
  public ShootNote(Shooter shooter, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, collector);
    this.shooter = shooter;
    this.collector = collector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO - specify speed from caller
    this.shooter.shoot18();
    this.startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((startTime + SHOOTER_WARM_UP_TIME_MILLIS) > System.currentTimeMillis()) {
      this.collector.in();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
