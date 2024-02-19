package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import java.util.Optional;

public class ShooterDynamics {
  private static final Pose3d SPEAKER_POSE = new Pose3d();
  private static final double SPEAKER_WIDTH_METERS = Units.inchesToMeters(41.375);

  private static final double NOTE_WIDTH_METERS = Units.inchesToMeters(14.0);
  private static final double MAX_NOTE_DRIFT_RAD = Math.toRadians(2.0);

  // Max distance the shooter can physically attain with a still trackable trajectory (2D)
  private static final double MAX_SHOOTER_DISTANCE = Units.feetToMeters(25.0); // TODO
  private static final double OTHER_ALLIANCE_WING_LINE = Units.inchesToMeters(420.0);

  private static final double ROBOT_SIZE_RADIUS = Units.inchesToMeters(25.0);

  public static Pair<Rotation2d, ShooterState> getRobotState(
      Pose2d robotPose, Vector<N2> robotVel) {
    return null;
  }

  public static Optional<Pair<Rotation2d, ShooterState>> calculateSpeaker(
      Vector<N2> robotVel, Pose2d robotPose) {
    // Ensure robot pose can legally shoot from there
    if (robotPose.getX() + ROBOT_SIZE_RADIUS >= OTHER_ALLIANCE_WING_LINE) return Optional.empty();

    // Calculate the position of the pivot of the robot on the field
    var pivotPose = getPivotPoseField(robotPose);

    // Calculate the distance between the pivot of the shooter and the center mark of the speaker in
    // two-dimensional space
    var distToTarget =
        Math.hypot(pivotPose.getX() - pivotPose.getX(), pivotPose.getY() - robotPose.getY());

    // Ensure that the shooter could possibly make the shot with a reasonable trajectory
    if (distToTarget > MAX_SHOOTER_DISTANCE) return Optional.empty();

    // TODO calculate note intersection points. Should allow for bank-shots on the inside sides of
    // the speaker as well.
    // If the note cant make it in at all angles within the angle bound, return empty Optional.

    double x = robotPose.getX();
    double y = robotPose.getY();

    // TODO get hub coordinates
    double hubX = 0;
    double hubY = 0;

    Translation2d pose = robotPose.getTranslation();

    double d = pose.getDistance(new Translation2d(hubX, hubY));

    // Calculate angle of shot based on distance - simple exponential regression
    double a = 0; // TODO get regression params
    double b = 0; // TODO get regression params
    double c = 0; // TODO get regression params
    double shotAngle = a * Math.pow(Math.E, -b * d) + c;

    // Calculate velocity of shot based on distance and angle - taken from AMB Robotics
    double SpeakerOpeningRadians = 0.244346;
    double velocity =
        1
            / Math.cos(shotAngle)
            * Math.sqrt(
                9.80665 * d / Math.abs(Math.tan(SpeakerOpeningRadians) - Math.tan(shotAngle)));

    // Calculate aim offset by multiplying robot y velocity by the airtime of the shot
    double airTime = d / velocity;
    double offsetY = robotVel.getData()[1] * airTime;
    hubY -= offsetY;

    // Calculate angle of robot based on robot position and aim location
    Rotation2d botAngle = Rotation2d.fromRadians(-Math.atan(x - hubX / y - hubY));

    // Offset shooter velocity by robot's x velocity
    velocity += robotVel.getData()[0];

    ShooterState shooterState = new ShooterState(Rotation2d.fromRadians(shotAngle), velocity);

    Optional<Pair<Rotation2d, ShooterState>> pair = Optional.of(new Pair(botAngle, shooterState));

    return pair;
  }

  // /**
  //  * Calculates a scalar value of how much of a bias should be placed on a given pose to make a
  // speaker shot
  //  *
  //  * @param robotPose the position to calculate for.
  //  * @return bias coefficient.
  //  */
  // private double calculateSpeakerPoseBias(Pose2d robotPose, Vector<N2> robotVel) {
  //
  //
  //
  //
  //
  // }

  private static Pose3d getPivotPoseField(Pose2d robotPose) {
    return new Pose3d(robotPose)
        .transformBy(
            new Transform3d(
                Constants.Shooter.PIVOT_POSE.getTranslation(),
                Constants.Shooter.PIVOT_POSE.getRotation()));
  }

  /**
   * Represent the state of the shooter at a given time
   *
   * @param angle the angle of the shooter relative to the plane intersecting the erector parallel
   *     to the ground.
   * @param shooterVelocityMetersPerSecond the velocity of the shooter's flywheels in meters per
   *     second. Positive values represent shooting the note out while negative values draw notes
   *     in.
   */
  public record ShooterState(Rotation2d angle, double shooterVelocityMetersPerSecond) {
    public static final ShooterState STARTING_STATE =
        new ShooterState(Rotation2d.fromDegrees(70), 0);
    public static final ShooterState TRAVEL_STATE = new ShooterState(Rotation2d.fromDegrees(35), 0);

    @Override
    public boolean equals(Object obj) {
      if (obj instanceof ShooterState other) {
        // The shooter and kickup percentages don't need to have super high accuracy in precision
        return Math.hypot(
                    angle.getCos() - other.angle.getCos(), angle.getSin() - other.angle.getSin())
                < 1e-3
            && Math.abs(shooterVelocityMetersPerSecond - other.shooterVelocityMetersPerSecond)
                <= 1e-2;
      }
      return false;
    }
  }
}
