// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkFlex topMotor;
  private final CANSparkFlex bottomMotor;
  private final double speed = 1;

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new CANSparkFlex(Constants.ShooterConstants.topMotor, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(Constants.ShooterConstants.bottomMotor, MotorType.kBrushless);
    topMotor.follow(bottomMotor, true);

    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);
    // TODO - do we need to invert one of these?
  }

  public void shootOn() {
    // TODO - direction might be wrong WRT real robot
    bottomMotor.set(-speed);
  };

  public void shootOff() {
    bottomMotor.stopMotor();
  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
