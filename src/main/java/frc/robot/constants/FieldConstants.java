package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil.AllianceRelative;
import java.util.Map;

/**
 * Constants regarding the positions of waypoints on the field. All dimensions are relative to the
 * <a
 * href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">always
 * blue origin</a> convention
 */
public class FieldConstants {
  public static final AprilTagFieldLayout FIELD_LAYOUT;

  static {
    FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  public static class Speaker {
    private static final Translation3d topRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(83.091));

    private static final Translation3d topLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    private static final Translation3d bottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    private static final Translation3d bottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    public static final AllianceRelative<Translation3d> centerSpeaker =
        AllianceRelative.from(bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5));
  }

  public static class Field {
    // Position of the spikes (note starting positions) on the field. Numbered from left to right
    // from driver station view (blue alliance).
    public static final Map<String, AllianceRelative<Translation2d>> spikePositions =
        Map.of(
            "Close One",
                AllianceRelative.from(
                    new Translation2d(
                        Units.inchesToMeters(113.885000), Units.inchesToMeters(274.562159))),
            "Close Two",
                AllianceRelative.from(
                    new Translation2d(
                        Units.inchesToMeters(113.885000), Units.inchesToMeters(217.562159))),
            "Close Three",
                AllianceRelative.from(
                    new Translation2d(
                        Units.inchesToMeters(113.885000), Units.inchesToMeters(160.562159))),
            "Middle One",
                AllianceRelative.from(
                    new Translation2d(
                        Units.inchesToMeters(325.486250), Units.inchesToMeters(292.562159))),
            "Middle Two",
                AllianceRelative.from(
                    new Translation2d(
                        Units.inchesToMeters(325.486250), Units.inchesToMeters(226.562159))),
            "Middle Three",
                AllianceRelative.from(
                    new Translation2d(
                        Units.inchesToMeters(325.486250), Units.inchesToMeters(160.562159))),
            "Middle Four",
                AllianceRelative.from(
                    new Translation2d(
                        Units.inchesToMeters(325.486250), Units.inchesToMeters(94.562159))),
            "Middle Five",
                AllianceRelative.from(
                    new Translation2d(
                        Units.inchesToMeters(325.486250), Units.inchesToMeters(28.562159))));

    public static final double STARTING_LINE_X = Units.inchesToMeters(229.076073);
    public static final double WING_LINE_X = Units.inchesToMeters(229.076073);
  }

  public static class Source {
    // Translation 3d of each opening
    public static final AllianceRelative<Translation3d> SOURCE_LEFT_OPENING =
        AllianceRelative.from(
            new Translation3d(
                Units.inchesToMeters(637.127924),
                Units.inchesToMeters(33.386440),
                Units.inchesToMeters(39.664118)));
    public static final AllianceRelative<Translation3d> SOURCE_MIDDLE_OPENING =
        AllianceRelative.from(
            new Translation3d(
                Units.inchesToMeters(615.472846),
                Units.inchesToMeters(20.894138),
                Units.inchesToMeters(39.664118)));
    public static final AllianceRelative<Translation3d> SOURCE_RIGHT_OPENING =
        AllianceRelative.from(
            new Translation3d(
                Units.inchesToMeters(593.817769),
                Units.inchesToMeters(8.401835),
                Units.inchesToMeters(39.664118)));
  }
}
