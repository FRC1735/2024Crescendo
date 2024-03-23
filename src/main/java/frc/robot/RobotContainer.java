package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.RotateAxel;
import frc.robot.commands.ShootNote;
import frc.robot.commands.StartShooter;
import frc.robot.subsystems.Axel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve-2024"));
  private final Collector collector = new Collector();
  private final Shooter shooter = new Shooter();
  private final Axel axel = new Axel();
  private final Climber climber = new Climber();
  private final Lighting lighting = new Lighting();


  // Commands
  private final PickUpNote pickUpNoteCommand = new PickUpNote(collector);
  private final ShootNote shootFullSpeedCommand = new ShootNote(shooter, collector,
      Constants.ShooterConstants.FULL_VELOCITY);
  private final ShootNote shootAmpSpeedCommand = new ShootNote(shooter, collector,
      Constants.ShooterConstants.AMP_VELOCITY);
  private final RotateAxel rotateAxelToCollect = new RotateAxel(axel, 0.2);
  private final RotateAxel rotateAxelForSpeakerShotUpAgainstSpeaker = new RotateAxel(axel, 0.255);
  private final RotateAxel rotateAxelForSpeakerShotMidzone = new RotateAxel(axel, 0.31);
  private final RotateAxel rotateAxelTest = new RotateAxel(axel, 0.3);
  private final RotateAxel rotateAxelLeftSideShoot = new RotateAxel(axel, 0.25);
  private final RotateAxel rotateAxelForAmp = new RotateAxel(axel, 0.26);
  private final StartShooter startShooter = new StartShooter(shooter, Constants.ShooterConstants.FULL_VELOCITY);
  // private final Command gyroOffset = new InstantCommand(drive::gyroOffset,
  // drive);

  // Controllers
  XboxController driverController = new XboxController(0);
  XboxController operaController = new XboxController(1);

  // Choosers
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    // Register PathPlanner Commands
    NamedCommands.registerCommand("PickUpNote", pickUpNoteCommand);
    NamedCommands.registerCommand("ShootNote", shootFullSpeedCommand);
    NamedCommands.registerCommand("ShootAmpNote", shootAmpSpeedCommand);
    NamedCommands.registerCommand("AxelAimSpeaker", rotateAxelForSpeakerShotUpAgainstSpeaker);
    NamedCommands.registerCommand("ResetGyro", new InstantCommand(drive::zeroGyro, drive));
    NamedCommands.registerCommand("RotateAxelToCollect", rotateAxelToCollect);
    NamedCommands.registerCommand("LeftSideSpeekerShootAnge", pickUpNoteCommand);
    NamedCommands.registerCommand("rotateAxelLeftSideShoot", rotateAxelLeftSideShoot);
    NamedCommands.registerCommand("rotateAxelForSpeakerShotMidzone", rotateAxelForSpeakerShotMidzone);
    NamedCommands.registerCommand("startShooter", startShooter);
    // NamedCommands.registerCommand("gyroOffset", gyroOffset);

    // Generate autoChooser for all PathPlanner commands
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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

    
    Command altDriveCommand = drive.driveCommand(
        () -> -MathUtil.applyDeadband(driverController.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> {
          return -driverController.getRightX();
        });

    //drive.setDefaultCommand(driveFieldOrientedDirectAngle);
    drive.setDefaultCommand(altDriveCommand);
    drive.zeroGyro();

    lighting.on();
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
    configureOperatorController();
  }

  private void configureDriverController() {
    // right bumper (6) -> right bumper on Xbox Controller
    new JoystickButton(driverController, 6).onTrue((new InstantCommand(drive::zeroGyro)));

    // left bumper (5) -> shoot at 20% speed
    new JoystickButton(driverController, 5)
      .whileTrue(
        new SequentialCommandGroup(
          rotateAxelForAmp,
          new ShootNote(shooter, collector, ShooterConstants.AMP_VELOCITY)
        )
      );

    // b (2) -> collect note with sensor enabeled
    new JoystickButton(driverController, 2).whileTrue(new PickUpNote(collector));

    // x (3) -> aim at speaker from black line, shoot note (100% speed)
    /*
    new JoystickButton(driverController, 3)
        .whileTrue(
            new SequentialCommandGroup(
                rotateAxelForSpeakerShotMidzone,
                new ShootNote(shooter, collector, ShooterConstants.FULL_VELOCITY)));
*/
    new JoystickButton(driverController, 3)
      .whileTrue(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            rotateAxelForSpeakerShotMidzone,
            new StartShooter(shooter, ShooterConstants.FULL_VELOCITY)
          ),
          new InstantCommand(collector::in, collector)
        )
      ).onFalse(new ParallelCommandGroup(
        new InstantCommand(shooter::stop, shooter),
        new InstantCommand(collector::stop, collector)
      ));            

    



    // a (1) -> position directly in front of speaker, shoot note (100% speed)
    /*
    new JoystickButton(driverController, 1)
        .whileTrue(
            new SequentialCommandGroup(
                rotateAxelForSpeakerShotUpAgainstSpeaker,
                new ShootNote(shooter, collector, ShooterConstants.FULL_VELOCITY)));
    */
    new JoystickButton(driverController, 1)
      .whileTrue(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            rotateAxelForSpeakerShotUpAgainstSpeaker,
            new StartShooter(shooter, ShooterConstants.FULL_VELOCITY)
          ),
          new InstantCommand(collector::in, collector)
        )
      ).onFalse(new ParallelCommandGroup(
        new InstantCommand(shooter::stop, shooter),
        new InstantCommand(collector::stop, collector)
      ));

    // y (4) -> shoot full speed
    new JoystickButton(driverController, 4)
        .whileTrue(new ShootNote(shooter, collector, ShooterConstants.FULL_VELOCITY));
  }

  private void configureOperatorController() {
    new JoystickButton(operaController, 6)
    .onTrue(new InstantCommand(collector::in, collector))
    .onFalse(new InstantCommand(collector::stop, collector));

    // up button -> make axel go up
    new POVButton(operaController, 180).whileTrue(new InstantCommand(axel::up,
        axel))
        .onFalse(new InstantCommand(axel::stop, axel));

    // down button -> make axel go down
    new POVButton(operaController,  0).whileTrue(new InstantCommand(axel::down,
        axel))
        .onFalse(new InstantCommand(axel::stop, axel));

    // 

    // y (4) -> retracting climber
    new JoystickButton(operaController, 4).whileTrue(new InstantCommand(climber::retract,
        climber))
        .onFalse(new InstantCommand(climber::stop, climber));

    // a (1) -> extending climber
    new JoystickButton(operaController, 1).onTrue(new InstantCommand(climber::extend,
        climber))
        .onFalse(new InstantCommand(climber::stop, climber));

    // b (2) -> Reverse Collector
    new JoystickButton(operaController, 2).onTrue(new InstantCommand(collector::out,
        collector))
        .onFalse(new InstantCommand(collector::stop, collector));

    // x (3) -> Reverse Shooter and Collector
    new JoystickButton(operaController, 3).onTrue(
        new ParallelCommandGroup(
            new InstantCommand(collector::out, collector),
            new InstantCommand(shooter::reverse, shooter)))
        .onFalse(
            new ParallelCommandGroup(
                new InstantCommand(collector::stop, collector),
                new InstantCommand(shooter::stop, shooter)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void stopAllSubsystems() {
    collector.stop();
    shooter.stop();
    climber.stop();
  }
}
