// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Axel extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(Constants.AxelConstants.leftMotor, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(Constants.AxelConstants.rightMotor, MotorType.kBrushless);
  private final SparkAbsoluteEncoder absoluteEncoder;
  private SparkPIDController pidController;
  private final double speed = 0.1; // TODO - increase!

  /** Creates a new Axel. */
  public Axel() {
    // rightMotor has absolute encoder attached to it

    leftMotor.follow(rightMotor);
    // TODO - do we need to invert one of these?

    absoluteEncoder = rightMotor.getAbsoluteEncoder();
    // TODO - set zero offset and inverted in Rev Hardware Client? This seems better

    // TODO - none of these values are real
    // absoluteEncoder.setInverted(false);
    // absoluteEncoder.setZeroOffset(.57);

    pidController = rightMotor.getPIDController();

    // TODO - these are not real values
    pidController.setP(1.2);
    pidController.setI(0);
    pidController.setD(0.2);
    pidController.setFF(0);

    pidController.setOutputRange(-speed, speed);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void pickUp() {
    // TODO - fake value
    pidController.setReference(0, ControlType.kPosition);
  };

  public void speaker() {
    // TODO - fake value
    pidController.setReference(0, ControlType.kPosition);
  };

  public void amp() {
    // TODO - fake value
    pidController.setReference(0, ControlType.kPosition);
  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Axel Encoder", absoluteEncoder.getPosition());
  }
}
