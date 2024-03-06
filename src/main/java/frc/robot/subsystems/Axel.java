// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.utils.MathUtils;

public class Axel extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(Constants.AxelConstants.leftMotor, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(Constants.AxelConstants.rightMotor, MotorType.kBrushless);
  private final SparkAbsoluteEncoder absoluteEncoder;
  private SparkPIDController pidController;
  private final double speed = 1; // TODO - increase!
  private double lastKnownP = 0;
  private double lastKnownI = 0;
  private double lastKnownD = 0;

  // TODO
  private double topEncoderLimit = 0.36;
  private double bottomEncoderLimit = 0.21;

  /** Creates a new Axel. */
  public Axel() {
    // rightMotor has absolute encoder attached to it

    leftMotor.follow(rightMotor, true);
    // TODO - do we need to invert one of these?

    absoluteEncoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // TODO - set zero offset and inverted in Rev Hardware Client? This seems better

    // TODO - none of these values are real
    // absoluteEncoder.setInverted(false);
    // absoluteEncoder.setZeroOffset(.57);

    pidController = rightMotor.getPIDController();
    pidController.setPositionPIDWrappingEnabled(false);
    pidController.setFeedbackDevice(absoluteEncoder);

    // Pre-spring PID values, worked pretty well
    pidController.setP(1.399999976158142);
    pidController.setI(0.00010000001202570274);
    pidController.setD(0);
    pidController.setFF(0);

    if (RobotContainer.DEBUG) {
      lastKnownP = pidController.getP();
      lastKnownI = pidController.getI();
      lastKnownD = pidController.getD();

      SmartDashboard.putNumber("Axel - P", pidController.getP());
      SmartDashboard.putNumber("Axel - I", pidController.getI());
      SmartDashboard.putNumber("Axel - D", pidController.getD());
    }

    pidController.setOutputRange(-speed, speed);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void up() {
    double pos = absoluteEncoder.getPosition();
    if (pos + 0.03 < topEncoderLimit) {
      pidController.setReference(pos - 0.03, ControlType.kPosition);
    }
  }

  public void down() {
    double pos = absoluteEncoder.getPosition();
    if (pos - 0.03 > bottomEncoderLimit) {
      pidController.setReference(pos + 0.03, ControlType.kPosition);
    }
  }

  public void stop() {
    rightMotor.stopMotor();
  }

  public void pickUp() {
    // TODO - fake value
    pidController.setReference(0.2, ControlType.kPosition);
  };

  public void speaker() {
    // TODO - fake value
    pidController.setReference(0.4, ControlType.kPosition);
  };

  public void amp() {
    // TODO - fake value
    pidController.setReference(0.3, ControlType.kPosition);
  };

  @Override
  public void periodic() {
    if (RobotContainer.DEBUG) {
      SmartDashboard.putNumber("Axel Encoder", absoluteEncoder.getPosition());

      double sdP = SmartDashboard.getNumber("Axel - P", 0);
      double sdI = SmartDashboard.getNumber("Axel - I", 0);
      double sdD = SmartDashboard.getNumber("Axel - D", 0);

      if (MathUtils.compareDouble(sdP, lastKnownP) || MathUtils.compareDouble(sdI, lastKnownI)
          || MathUtils.compareDouble(sdD, lastKnownD)) {
        pidController.setP(sdP);
        pidController.setI(sdI);
        pidController.setD(sdD);
      }

      SmartDashboard.putNumber("Axel - P", pidController.getP());
      SmartDashboard.putNumber("Axel - I", pidController.getI());
      SmartDashboard.putNumber("Axel - D", pidController.getD());

      lastKnownP = pidController.getP();
      lastKnownI = pidController.getI();
      lastKnownD = pidController.getD();
    }
  }
}
