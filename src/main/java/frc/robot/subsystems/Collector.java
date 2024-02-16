// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(Constants.CollectorConstants.motor, MotorType.kBrushless);
  private final double speed = 0.1;

  /** Creates a new Collector. */
  public Collector() {
  }

  // TODO - direction might be wrong WRT real robot
  public void in() {
    motor.set(speed);
  };

  // TODO - direction migth be wrong WRT real robot
  public void out() {
    motor.set(-speed);
  };

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
