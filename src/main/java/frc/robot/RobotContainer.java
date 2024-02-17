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
        () -> MathUtil.applyDeadband(driverController.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> {
          double rightX = driverController.getRightX();
          // System.out.println("RightX: " + rightX);
          return rightX;
        },
        () -> {
          double rightY = driverController.getRightY();
          // System.out.println("RightY: " + rightY);
          return rightY;
        });

    Command driveFieldOrientedAnglularVelocity = drive.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRawAxis(2));

    // drive.setDefaultCommand(driveFieldOrientedDirectAngle);
    drive.setDefaultCommand(driveFieldOrientedAnglularVelocity);

  }

  private void configureBindings() {
    // 6 -> right bumper on Xbox Controller
    new JoystickButton(driverController, 6).onTrue((new InstantCommand(drive::zeroGyro)));

    //// SmartDashboard controls
    // Collector
    SmartDashboard.putData("Collector In", new InstantCommand(collector::in, collector));
    SmartDashboard.putData("Collector Out", new InstantCommand(collector::out, collector));
    SmartDashboard.putData("Collector Stop", new InstantCommand(collector::stop, collector));

    // Shooter
    SmartDashboard.putData("Shooter On", new InstantCommand(shooter::shootOn, shooter));
    SmartDashboard.putData("Shooter Off", new InstantCommand(shooter::shootOff, shooter));

    // Axel
    SmartDashboard.putData("Axel Pickup", new InstantCommand(axel::pickUp, axel));
    SmartDashboard.putData("Axel Amp", new InstantCommand(axel::amp, axel));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
