// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  private Shooter shooter;
  private Collector collector;
  private int velocity;
  private boolean isShooting;
  private Timer timer;

  /** Creates a new ShootNote. */
  public ShootNote(Shooter shooter, Collector collector, int velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, collector);
    this.shooter = shooter;
    this.collector = collector;
    this.velocity = velocity;
    this.isShooting = false;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooter.shoot(velocity);
    this.timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO - close enough, we should tune shooter PID further and check when we
    // reach the setpoint
    if (shooter.getAverageVelocity() < (velocity + 200)) {
      this.collector.in();
      if (isShooting == false) {
        this.timer.reset();
      }
      this.isShooting = true;
      this.timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    collector.stop();
    this.isShooting = false;
    this.timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isShooting && timer.hasElapsed(1);
  }
}
