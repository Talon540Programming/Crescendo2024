package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

public final class Constants {
  private static RobotType kRobotType = RobotType.ROBOT_SIMBOT;
  // Allows tunable values to be changed when enabled. Also adds tunable selectors to AutoSelector
  public static final boolean TUNING_MODE = true;
  // Disable the AdvantageKit logger from running
  public static final boolean ENABLE_LOGGING = true;

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
    if (getRobotType() == RobotType.ROBOT_SIMBOT) return RobotMode.SIM;
    else return RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;
  }

  public static class Shooter {
    public static final Rotation2d MIN_SHOOTER_ANGLE = Rotation2d.fromDegrees(27.5);
    public static final Rotation2d MAX_SHOOTER_ANGLE = Rotation2d.fromDegrees(120.0);
    public static final double SHOOTER_LENGTH_METERS = Units.inchesToMeters(20.75);
    // Position of the pivot point of the shooter in the RCS
    public static final Pose3d PIVOT_POSE =
        new Pose3d(
            Units.inchesToMeters(-2.5),
            0,
            Units.inchesToMeters(5.727224),
            new Rotation3d(0.0, 0.0, Math.toRadians(180.0)));
  }

  public static class Vision {
    public enum CameraType {
      AprilTagCamera
    }

    public record CameraConfig(
        String cameraName, Transform3d robotToCamera, CameraType cameraType) {}

    public static final List<CameraConfig> configs =
        List.of(
            new CameraConfig(
                "Cam0",
                new Transform3d(
                    new Translation3d(0.279, 0.281, 0.272),
                    new Rotation3d(
                        Units.degreesToRadians(0), //TODO
                        Units.degreesToRadians(45),
                        Units.degreesToRadians(22.5))),
                CameraType.AprilTagCamera),
            new CameraConfig(
                "Cam1",
                new Transform3d(
                    new Translation3d(0.281, -0.279, 0.272),
                    new Rotation3d(
                        Units.degreesToRadians(0),//TODO
                        Units.degreesToRadians(45),
                        Units.degreesToRadians(67.5))),
                CameraType.AprilTagCamera),
            new CameraConfig(
                "Cam2",
                new Transform3d(
                    new Translation3d(-0.285, 0.284, 0.273),
                    new Rotation3d(
                        Units.degreesToRadians(0),//TODO
                        Units.degreesToRadians(20),
                        Units.degreesToRadians(-45))),
                CameraType.AprilTagCamera),
            new CameraConfig(
                "Cam3",
                new Transform3d(
                    new Translation3d(-0.33, 0.139, 0.157),
                    new Rotation3d(
                        Units.degreesToRadians(0),//TODO
                        Units.degreesToRadians(30),
                        Units.degreesToRadians(0))),
                CameraType.AprilTagCamera));
  }

  public static class Intake {
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromRadians(0.635);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(125);

    public static final Rotation2d GROUND_INTAKE_ANGLE = Rotation2d.fromRadians(0.635);
    public static final Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(125);

    public static final double kIntakeLengthMeters = Units.inchesToMeters(13.5); // TODO
  }
}
