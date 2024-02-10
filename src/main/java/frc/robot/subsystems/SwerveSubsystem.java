package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// Based on: https://github.com/BroncBotz3481/YAGSL-Example/blob/main/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java
// Notice all the telemetry, pathplanner, etc. We should do that once things are running at a basic level.

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    public double maximumSpeed = Units.feetToMeters(14.5);

    public SwerveSubsystem(File directory) {
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        /*
         * AutoBuilder.configureHolonomic(
         * this::getPose,
         * this::resetOdometry,
         * this::getRobotOrientedVelocity,
         * this::setChassisSpeed,
         * new HolonomicPathFollowerConfig(
         * new PIDConstants(Constants.kSwerveAutoPIDP, Constants.kSwerveAutoPIDI,
         * Constants.kSwerveAutoPIDD),
         * new PIDConstants(
         * swerveDrive.swerveController.config.headingPIDF.p,
         * swerveDrive.swerveController.config.headingPIDF.i,
         * swerveDrive.swerveController.config.headingPIDF.d),
         * Constants.kMaxModuleSpeed,
         * Units.feetToMeters(Constants.kDriveBaseRadius),
         * new ReplanningConfig()),
         * this::shouldPathFlip,
         * this);
         */

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        return run(() -> {
            double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
            double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(
                    new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                            Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                    true,
                    false);
        });
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotOrientedVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public void setChassisSpeed(ChassisSpeeds velocity) {
        swerveDrive.setChassisSpeeds(velocity);
    }

    public boolean shouldPathFlip() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}