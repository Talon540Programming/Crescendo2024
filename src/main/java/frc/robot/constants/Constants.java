package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.nio.file.Path;
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
    public record CameraConfig(
        String cameraName, Transform3d robotToCamera, Path calibrationPath) {}

    public static final List<CameraConfig> configs =
        List.of(
            new CameraConfig(
                "UNDER_SHOOTER",
                new Transform3d(
                    -0.330312, 0.138773, 0.157061, new Rotation3d(0, Math.toRadians(-30), Math.PI)),
                Path.of("camera_calibrations/photon_calibration_UNDER_SHOOTER_1280x720.json")),
            new CameraConfig(
                "BACK_LEFT",
                new Transform3d(
                    -0.285206,
                    0.283806,
                    0.272624,
                    new Rotation3d(0, Math.toRadians(-20), Math.toRadians(135))),
                Path.of("camera_calibrations/photon_calibration_BACK_LEFT_1280x720.json")),
            new CameraConfig(
                "FRONT_LEFT",
                new Transform3d(
                    0.278740,
                    0.280968,
                    0.272098,
                    new Rotation3d(0, Math.toRadians(-45), Math.toRadians(-22.5))),
                Path.of("camera_calibrations/photon_calibration_FRONT_LEFT_1280x720.json")),
            new CameraConfig(
                "FRONT_RIGHT",
                new Transform3d(
                    0.281034,
                    -0.278751,
                    0.272098,
                    new Rotation3d(0, Math.toRadians(-45), Math.toRadians(67.5))),
                Path.of("camera_calibrations/photon_calibration_FRONT_RIGHT_1280x720.json")));
  }

  public static class Intake {
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromRadians(0.635);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(125);

    public static final Rotation2d GROUND_INTAKE_ANGLE = Rotation2d.fromRadians(0.635);
    public static final Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(125);

    public static final double kIntakeLengthMeters = Units.inchesToMeters(13.5); // TODO
  }
}
