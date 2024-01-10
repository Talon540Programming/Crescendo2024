package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  private static RobotType kRobotType = RobotType.ROBOT_2024_COMP;
  // Allows tunable values to be changed when enabled
  public static final boolean TUNING_MODE = true;

  public static final double kLoopPeriodSecs = 0.02;

  public enum RobotMode {
    REAL,
    SIM,
    REPLAY
  }

  public enum RobotType {
    ROBOT_2024_COMP,
    ROBOT_SIMBOT
  }

  public static RobotType getRobotType() {
    if (RobotBase.isReal() && kRobotType == RobotType.ROBOT_SIMBOT) {
      DriverStation.reportError(
          "Robot is set to SIM but it isn't a SIM, setting it to Competition Robot as redundancy.",
          false);
      kRobotType = RobotType.ROBOT_2024_COMP;
    }

    if (RobotBase.isSimulation() && kRobotType != RobotType.ROBOT_SIMBOT) {
      DriverStation.reportError(
          "Robot is set to REAL but it is a SIM, setting it to SIMBOT as redundancy.", false);
      kRobotType = RobotType.ROBOT_SIMBOT;
    }

    return kRobotType;
  }

  public static RobotMode getRobotMode() {
    return switch (getRobotType()) {
      case ROBOT_2024_COMP -> RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;
      case ROBOT_SIMBOT -> RobotMode.SIM;
    };
  }

  public static class Drivetrain {
    // L2 gearing
    public static final double kDriveGearing = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double kTurnGearing = 12.8;

    public static final boolean kTurnMotorInverted = false;

    public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);

    public static final double kTrackWidthXMeters = 0; // TODO
    public static final double kTrackWidthYMeters = 0; // TODO
    public static final double kDriveBaseRadiusMeters =
        Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);

    public static final double kMaxLinearVelocityMetersPerSecond = Units.feetToMeters(14.5);
    public static final double kMaxAngularVelocityRadiansPerSecond =
        kMaxLinearVelocityMetersPerSecond / kDriveBaseRadiusMeters;

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(
            Constants.Drivetrain.kTrackWidthXMeters / 2.0,
            Constants.Drivetrain.kTrackWidthYMeters / 2.0),
        new Translation2d(
            Constants.Drivetrain.kTrackWidthXMeters / 2.0,
            -Constants.Drivetrain.kTrackWidthYMeters / 2.0),
        new Translation2d(
            -Constants.Drivetrain.kTrackWidthXMeters / 2.0,
            Constants.Drivetrain.kTrackWidthYMeters / 2.0),
        new Translation2d(
            -Constants.Drivetrain.kTrackWidthXMeters / 2.0,
            -Constants.Drivetrain.kTrackWidthYMeters / 2.0)
      };
    }

    public static final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(getModuleTranslations());
  }
}
