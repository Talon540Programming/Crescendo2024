package frc.robot.subsystems.shooter.dynamics;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import java.util.Optional;

public class ShooterDynamics {
  private static final double G = 9.80665;

  private static final Pose3d SPEAKER_POSE = new Pose3d(); // TODO
  private static final double SPEAKER_WIDTH_METERS = Units.inchesToMeters(41.375);
  private static final double SPEAKER_OPENING_ANGLE_RAD = Math.toRadians(14.0);

  private static final double NOTE_WIDTH_METERS = Units.inchesToMeters(14.0);
  private static final double MAX_NOTE_DRIFT_RAD = Math.toRadians(2.0);
  private static final double HUB_WIDTH_METERS = 1.05;

  // Max distance the shooter can physically attain with a still trackable trajectory (2D)
  private static final double MAX_SHOOTER_DISTANCE = Units.feetToMeters(25.0); // TODO
  private static final double OTHER_ALLIANCE_WING_LINE = Units.inchesToMeters(420.0);

  private static final double ROBOT_SIZE_RADIUS = Units.inchesToMeters(25.0);

  // public static Pair<Rotation2d, ShooterState> getRobotState(
  //     Pose2d robotPose, Vector<N2> robotVel) {
  //   return null;
  // }

  // public static Optional<Pair<Rotation2d, ShooterState>> calculateSpeaker(
  //     Vector<N2> robotVel, Pose2d robotPose) {
  //   // Ensure robot pose can legally shoot from there
  //   if (robotPose.getX() + ROBOT_SIZE_RADIUS >= OTHER_ALLIANCE_WING_LINE) return Optional.empty();
  //
  //   // Calculate the position of the pivot of the shooter relative to the field
  //   var pivotPose =
  //       new Pose3d(robotPose)
  //           .transformBy(
  //               new Transform3d(
  //                   Constants.Shooter.PIVOT_POSE.getTranslation(),
  //                   Constants.Shooter.PIVOT_POSE.getRotation()));
  //
  //   // Calculate the distance between the pivot of the shooter and the center mark of the speaker in
  //   // two-dimensional space
  //   var distToTarget =
  //       Math.hypot(pivotPose.getX() - pivotPose.getX(), pivotPose.getY() - robotPose.getY());
  //
  //   // Ensure that the shooter could possibly make the shot with a reasonable trajectory
  //   if (distToTarget > MAX_SHOOTER_DISTANCE) return Optional.empty();
  //
  //   // Calculate angle of shot based on distance
  //   double shooterAngleRad = calculateShooterAngle(distToTarget);
  //
  //   // Calculate the required velocity of the shot
  //   // https://ambcalc.com/docs/projectile.pdf
  //   double shooterVelocityMetersPerSecond =
  //       1
  //           / Math.cos(shooterAngleRad)
  //           * Math.sqrt(
  //               9.80665
  //                   * distToTarget
  //                   / Math.abs(Math.tan(SPEAKER_OPENING_ANGLE_RAD) - Math.tan(shooterAngleRad)));
  //
  //
  //   // Calculate aim offset by multiplying robot y velocity by the airtime of the shot
  //   double airTime = distToTarget / shooterVelocityMetersPerSecond;
  //   double offsetY = robotVel.getData()[1] * airTime;
  //   hubY -= offsetY;
  //
  //   // Calculate angle of robot based on robot position and aim location
  //   Rotation2d botAngle = Rotation2d.fromRadians(-Math.atan(x - hubX / y - hubY));
  //
  //   // Ensure the apparent width of the hub from the robot's vantage point is large enough for the
  //   // note
  //   double apparentHubWidthMeters = HUB_WIDTH_METERS * Math.cos(botAngle.getRadians());
  //   if (apparentHubWidthMeters + 0.1 < NOTE_WIDTH_METERS) return Optional.empty();
  //
  //   // Offset shooter velocity by robot's x velocity
  //   velocity += robotVel.getData()[0];
  //
  //   ShooterState shooterState = new ShooterState(Rotation2d.fromRadians(shotAngle), velocity);
  //
  //   return Optional.of(Pair.of(botAngle, shooterState));
  // }
  //
  // private static double calculateShooterAngle(double distanceToTarget) {
  //   double a = 0; // TODO determine regression params
  //   double b = 0; // TODO determine regression params
  //   double c = 0; // TODO determine regression params
  //
  //   return a * Math.pow(Math.E, -b * distanceToTarget) + c;
  // }
}
