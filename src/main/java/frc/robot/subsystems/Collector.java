// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.sensors.DistanceSensor;

public class Collector extends SubsystemBase {
  private final CANSparkMax motor;
  private final double speed = 1;
  private final DistanceSensor distanceSensor;

  /** Creates a new Collector. */
  public Collector() {
    motor = new CANSparkMax(Constants.CollectorConstants.motor, MotorType.kBrushless);
    distanceSensor = new DistanceSensor(0);
  }

  public void shoot() {
    motor.set(-speed);
  }

  public void in() {
    motor.set(-speed);
  };

  public void out() {
    motor.set(speed);
  };

  public void stop() {
    motor.stopMotor();
  }

  public boolean noteReadyToShoot() {
    if (distanceSensor.getDistance() < 15) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.DEBUG) {
      SmartDashboard.putNumber("Distance Sensor", distanceSensor.getDistance());
    }
  }
}
