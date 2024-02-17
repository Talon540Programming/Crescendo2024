package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

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

  // // TODO
  // public static ShooterState calculateSpeaker(Vector<N2> robotVel, Pose2d robotPose) {
  //   return null;
  // }

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

  public boolean isValidRobotState(Pose2d robotPose, Vector<N2> robotVel) {
    // Ensure robot pose can legally shoot from there
    if (robotPose.getX() + ROBOT_SIZE_RADIUS >= OTHER_ALLIANCE_WING_LINE) return false;

    // Calculate the position of the pivot of the robot on the field
    var pivotPose = getPivotPoseField(robotPose);

    // Calculate the distance between the pivot of the shooter and the center mark of the speaker in
    // two-dimensional space
    var distToTarget =
        Math.hypot(pivotPose.getX() - pivotPose.getX(), pivotPose.getY() - robotPose.getY());

    // Ensure that the shooter could possibly make the shot with a reasonable trajectory
    if (distToTarget > MAX_SHOOTER_DISTANCE) return false;

    // Calculate if a note with lateral drift would make it into the speaker
    double noteDriftLeft = 0.0; // TODO constant + function of velocity
    double noteDriftRight = 0.0; // TODO constant + function of velocity

    // TODO calculate note intersection points. Should allow for bank-shots on the inside sides of
    // the speaker as well.
    //  If the note cant make it in at all angles within the angle bound, return 0.

    return true;
  }

  private static Pose3d getPivotPoseField(Pose2d robotPose) {
    return new Pose3d(robotPose)
        .transformBy(
            new Transform3d(
                Constants.Shooter.PIVOT_POSE.getTranslation(),
                Constants.Shooter.PIVOT_POSE.getRotation()));
  }

  public record ShooterState(
      Rotation2d angle,
      double shooterVelocityMetersPerSecond,
      double kickupVelocityMetersPerSecond) {
    public static final ShooterState STARTING_STATE =
        new ShooterState(Rotation2d.fromDegrees(70), 0, 0);
    public static final ShooterState TRAVEL_STATE =
        new ShooterState(Rotation2d.fromDegrees(35), 0, 0);

    @Override
    public boolean equals(Object obj) {
      if (obj instanceof ShooterState other) {
        return Math.hypot(
                    angle.getCos() - other.angle.getCos(), angle.getSin() - other.angle.getSin())
                < 1.5
            && Math.abs(shooterVelocityMetersPerSecond - other.shooterVelocityMetersPerSecond)
                <= 1e-2
            && Math.abs(kickupVelocityMetersPerSecond - other.kickupVelocityMetersPerSecond)
                <= 1e-2;
      }
      return false;
    }
  }
}
