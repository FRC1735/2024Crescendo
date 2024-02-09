// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Axel extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(Constants.AxelConstants.leftMotor,MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(Constraints.AxelConstants.rightMotor,MotorType.kBrushless);
  /** Creates a new Axel. */
  public Axel() {}

 public void pickUp() {

 };


 public void speaker() {

 };

public void amp() {


};
  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
