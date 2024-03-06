// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final CANSparkFlex motor;
  private final double speed = 0.1;

  /** Creates a new Climber. */
  public Climber() {
    motor = new CANSparkFlex(Constants.ClimberConstants.motor, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
  }

  public void in() {
    // TODO - direction may be wrong
    motor.set(speed);
  }

  public void out() {
    // TODO - direction may be wrong
    motor.set(-speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
