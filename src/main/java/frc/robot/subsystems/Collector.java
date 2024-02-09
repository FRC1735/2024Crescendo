// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(Constraints.Collector.motor,kBrushless)
  /** Creates a new Collector. */
  public Collector() {}


  public void in () {

  };

  public void out () {

  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
