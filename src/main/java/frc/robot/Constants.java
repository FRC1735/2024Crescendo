package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class AutoConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AxelConstants {
    public static final int leftMotor = 1;
    public static final int rightMotor = 2;
  }

  public static final class ShooterConstants {
    public static final int topMotor = 3;
    public static final int bottomMotor = 4;

    public static final int FULL_VELOCITY = -8400;
    public static final int AMP_VELOCITY = -1180;
  }

  public static final class CollectorConstants {
    public static final int motor = 5;
  }

  public static final class ClimberConstants {
    public static final int motor = 6;
  }
}
