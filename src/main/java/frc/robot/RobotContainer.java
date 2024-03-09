package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Axel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve-2024"));
  private final Collector collector = new Collector();
  private final Shooter shooter = new Shooter();
  private final Axel axel = new Axel();
  private final Climber climber = new Climber();

  // Commands
  private final PickUpNote pickUpNoteCommand = new PickUpNote(collector);
  private final ShootNote shootFullSpeedCommand = new ShootNote(shooter, collector, Constants.ShooterConstants.FULL_VELOCITY);
  private final ShootNote shootAmpSpeedCommand =  new ShootNote(shooter, collector, Constants.ShooterConstants.AMP_VELOCITY);

  // Controllers
  XboxController driverController = new XboxController(0);

  public RobotContainer() {
    configureBindings();


    // Register PathPlanner Commands
    NamedCommands.registerCommand("PickUpNote", pickUpNoteCommand);
    NamedCommands.registerCommand("ShootNote", shootFullSpeedCommand);
    NamedCommands.registerCommand("ShootAmpNote", shootAmpSpeedCommand);



    // TODO - hook up to button state if we want this?
    boolean snapToRightAngleEnabled = false;

    Command driveFieldOrientedDirectAngle = drive.driveCommand(
        () -> -MathUtil.applyDeadband(driverController.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> {
          double rightX = driverController.getRightX();
          if (snapToRightAngleEnabled) {
            return snapToRightAngle(-rightX);
          } else {
            return -rightX;
          }
        },
        () -> {
          double rightY = driverController.getRightY();
          if (snapToRightAngleEnabled) {
            return snapToRightAngle(-rightY);
          } else {
            return -rightY;
          }
        });

    drive.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

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
    configureDriverController();
  }

  private void configureDriverController() {
    // right bumper (6) -> right bumper on Xbox Controller
    new JoystickButton(driverController, 6).onTrue((new InstantCommand(drive::zeroGyro)));

    // b (2) -> collect note with sensor enabeled
    new JoystickButton(driverController, 2).whileTrue(new PickUpNote(collector));

    // x (3) -> shoot note
    new JoystickButton(driverController, 3)
        .whileTrue(new ShootNote(shooter, collector, ShooterConstants.FULL_VELOCITY));

    new POVButton(driverController, 180).whileTrue(new InstantCommand(axel::up,
        axel))
        .onFalse(new InstantCommand(axel::stop, axel));

    new POVButton(driverController, 0).whileTrue(new InstantCommand(axel::down,
        axel))
        .onFalse(new InstantCommand(axel::stop, axel));

    // Climber control
    /*
     * new POVButton(driverController, 0).onTrue(new InstantCommand(climber::in,
     * climber))
     * .onFalse(new InstantCommand(climber::stop, climber));
     * 
     * new POVButton(driverController, 180).onTrue(new InstantCommand(climber::out,
     * climber))
     * .onFalse(new InstantCommand(climber::stop, climber));
     */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void stopAllSubsystems() {
    collector.stop();
    shooter.stop();
    climber.stop();
  }
}
