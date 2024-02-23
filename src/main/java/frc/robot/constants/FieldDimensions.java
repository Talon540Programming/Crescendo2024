package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/**
 * Constants regarding the positions of waypoints on the field. All dimensions are relative to the
 * <a
 * href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">always
 * blue origin</a> convention
 */
public class FieldDimensions {
  public static final Pose3d SPEAKER_CENTER_POSE = new Pose3d(); // TODO
  public static final Pose3d AMP_CENTER_POSE = new Pose3d(); // TODO

  public static final List<Pose2d> LEGAL_NOTE_POSITIONS = List.of(); // TODO

  public static final Pose2d FEEDER_STATION_BAY_1_GROUND_INTAKE = new Pose2d(); // TODO
  public static final Pose2d FEEDER_STATION_BAY_1_SHOOTER_INTAKE = new Pose2d(); // TODO

  public static final Pose2d FEEDER_STATION_BAY_2_GROUND_INTAKE = new Pose2d(); // TODO
  public static final Pose2d FEEDER_STATION_BAY_2_SHOOTER_INTAKE = new Pose2d(); // TODO

  public static final Pose2d FEEDER_STATION_BAY_3_GROUND_INTAKE = new Pose2d(); // TODO
  public static final Pose2d FEEDER_STATION_BAY_4_SHOOTER_INTAKE = new Pose2d(); // TODO

  public static final double WING_LINE_DISTANCE_METERS = Units.inchesToMeters(420.0);
  public static final double STARTING_ZONE_LINE_DISTANCE_METERS = Units.inchesToMeters(76.111250);
}
