// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private final boolean DEBUG = true;

  private final CANSparkFlex topMotor;
  private final CANSparkFlex bottomMotor;
  private RelativeEncoder bottomEncoder;
  private RelativeEncoder topEncoder;
  private SparkPIDController pidController;

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new CANSparkFlex(Constants.ShooterConstants.topMotor, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(Constants.ShooterConstants.bottomMotor, MotorType.kBrushless);
    topMotor.follow(bottomMotor, true);

    // NOTE: if you call CANSparkFlex.getEncoder() more than one time
    // NOTE: you get an illegal state exception
    // NOTE: _I think_ it is trying to reinitialize the encoder when it happens
    bottomEncoder = bottomMotor.getEncoder();
    topEncoder = topMotor.getEncoder();

    this.pidController = bottomMotor.getPIDController();
    pidController.setP(0.01);
    pidController.setI(0.0);
    pidController.setD(0);
    pidController.setIZone(0.0);
    pidController.setFF(0);
    pidController.setOutputRange(-1, 0);

    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);
  }

  public void shoot(int velocity) {
    pidController.setReference(velocity, ControlType.kVelocity);
  }

  public void stop() {
    bottomMotor.stopMotor();
  };

  public double getAverageVelocity() {
    return (topEncoder.getVelocity() + bottomEncoder.getVelocity() / 2);
  }

  @Override
  public void periodic() {
    if (DEBUG) {
      SmartDashboard.putNumber("Shooter - topVelocity", topEncoder.getVelocity());
      SmartDashboard.putNumber("Shooter - bottomVelocity", bottomEncoder.getVelocity());
    }
    SmartDashboard.putNumber("Shooter - Average Velocity", getAverageVelocity());
  }
}
