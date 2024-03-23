// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final boolean DEBUG = false;

  private final CANSparkFlex motor;
  private final double speed = 1;

  private RelativeEncoder encoder;

  /** Creates a new Climber. */
  public Climber() {
    motor = new CANSparkFlex(Constants.ClimberConstants.motor, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);

    encoder = motor.getEncoder();

    //motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    //motor.setSoftLimit(SoftLimitDirection.kForward, 0);
  }

  public void extend() {
    motor.set(-speed);
  }

  public void retract() {
    //motor.set(speed);
    
    if (encoder.getPosition() < -5) {
      motor.set(speed);
    } else {
      motor.set(0);
    }
    
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DEBUG) {
      SmartDashboard.putNumber("Climber Encoder", encoder.getPosition());
    }
  }
}
