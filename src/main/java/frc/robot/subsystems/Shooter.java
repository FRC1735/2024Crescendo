// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private final CANSparkFlex topMotor;
  private final CANSparkFlex bottomMotor;
  private final double speed = 1;
  private RelativeEncoder bottomEncoder;
  private RelativeEncoder topEncoder;

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

    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);
  }

  public void shoot50() {
    bottomMotor.set(-speed * 0.5);
  }

  public void shoot18() {
    bottomMotor.set(-speed * 0.18);
  }

  public void shootOn() {
    bottomMotor.set(-speed);
  };

  public void shootOff() {
    bottomMotor.stopMotor();
  };

  @Override
  public void periodic() {
    if (RobotContainer.DEBUG) {
      SmartDashboard.putNumber("topVelocity", topEncoder.getVelocity());
      SmartDashboard.putNumber("bottomVelocity", bottomEncoder.getVelocity());
    }
  }
}
