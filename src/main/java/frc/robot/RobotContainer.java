package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Axel;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve-2024"));
  private final Collector collector = new Collector();
  private final Shooter shooter = new Shooter();
  private final Axel axel = new Axel();

  // Controllers
  XboxController driverController = new XboxController(0);

  public static boolean DEBUG = true;

  public RobotContainer() {
    configureBindings();

    Command driveFieldOrientedDirectAngle = drive.driveCommand(
        () -> -MathUtil.applyDeadband(driverController.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> {
          double rightX = driverController.getRightX();

          SmartDashboard.putNumber("X", rightX);
          return snapToRightAngle(-rightX);
        },
        () -> {
          double rightY = driverController.getRightY();
          SmartDashboard.putNumber("Y", rightY);
          return snapToRightAngle(-rightY);
        });

    drive.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  // 0.96 -> 1
  // 0.99 -> 1
  // 0.88 -> 1
  // 0.60 -> 0.6
  // 0.04 -> 0
  // -0.04 -> 0

  private double snapToRightAngle(double i) {
    double snapThreshold = 0.2;

    if (i < 0 + snapThreshold && i > 0 - snapThreshold) {
      return 0;
    } else if (i > 1 - snapThreshold) {
      return 1;
    } else if (i < snapThreshold - 1) {
      return -1;
    } else {
      return i;
    }
  }

  private void configureBindings() {

    SmartDashboard.putData("Axel - Speaker", new InstantCommand(axel::speaker, axel));

    configureDriverController();
  }

  private void configureDriverController() {
    // 6 -> right bumper on Xbox Controller
    new JoystickButton(driverController, 6).onTrue((new InstantCommand(drive::zeroGyro)));

    // A -> swerve angle down (+ 1 is down)
    /*
     * new JoystickButton(driverController, 1)
     * .onTrue(
     * drive.driveCommand(
     * () -> -MathUtil.applyDeadband(driverController.getLeftY(),
     * OperatorConstants.LEFT_Y_DEADBAND),
     * () -> -MathUtil.applyDeadband(driverController.getLeftX(),
     * OperatorConstants.LEFT_X_DEADBAND),
     * () -> 0,
     * () -> 1));
     */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void stopAllSubsystems() {
    collector.stop();
    shooter.shootOff();
  }
}
