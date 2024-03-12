package frc.robot.subsystems.shooter.dynamics;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.util.GeomUtil;
import java.util.Optional;

public class ShooterDynamics {
  private static final double SHOOTING_RADIUS_METERS = Units.feetToMeters(15);
  private static final double INTAKE_RADIUS_METERS = Units.feetToMeters(5);

  private static final double SHOOTER_SPEAKER_YAW_ANGLE_TOLERANCE_DEGREES = 3.0;
  private static final double SHOOTER_INTAKE_VELOCITY = -10;
  private static final double SHOOTER_YEET_VELOCITY = 40;

  private static Translation3d calculatePivotTranslation(Pose2d robotPose) {
    return new Pose3d(robotPose)
        .transformBy(GeomUtil.toTransform3d(Constants.Shooter.PIVOT_POSE))
        .getTranslation();
  }

  public static Optional<ShooterState> calculateSpeakerState(
      Pose2d robotPose, ChassisSpeeds robotVel) {
    // Ensure shot is within legal limits
    if (robotPose.getX() > FieldConstants.Field.WING_LINE_X) {
      return Optional.empty();
    }

    // Ensure within shooting zone (ensures shots are higher accuracy)
    if (!inShooterZone(robotPose)) {
      return Optional.empty();
    }

    var pivotTranslation = calculatePivotTranslation(robotPose);
    var targetTranslation = FieldConstants.Speaker.centerSpeaker.get();

    // TODO, determine if linear approximation is enough or if regression is needed to determine the
    //  shooter angle
    // Calculate the pitch angle to the target
    var pivotToTargetAngle =
        Rotation2d.fromRadians(
            Math.atan(
                (targetTranslation.getZ() - pivotTranslation.getZ())
                    / pivotTranslation
                        .toTranslation2d()
                        .getDistance(targetTranslation.toTranslation2d())));

    return Optional.of(new ShooterState(pivotToTargetAngle, SHOOTER_YEET_VELOCITY));
  }

  public static Optional<ShooterState> calculateIntakeState(Pose2d robotPose) {
    // Ensure within intake zone (ensures shots are higher accuracy)
    if (!inIntakeZone(robotPose)) {
      return Optional.empty();
    }

    var pivotTranslation = calculatePivotTranslation(robotPose);

    // Determine distance to the source wall
    var upperPoint = FieldConstants.Source.SOURCE_LEFT_OPENING.get();
    var lowerPoint = FieldConstants.Source.SOURCE_RIGHT_OPENING.get();
    double a = lowerPoint.getY() - upperPoint.getY();
    double b = upperPoint.getX() - lowerPoint.getX();
    double c = upperPoint.getY() * lowerPoint.getX() - upperPoint.getX() * lowerPoint.getY();

    double distance2d =
        Math.abs(a * pivotTranslation.getX() + b * pivotTranslation.getY() + c) / Math.hypot(a, b);

    // TODO determine if this needs to be higher to account for initial note velocity
    double sourceHeight = upperPoint.getZ();

    var pivotToTargetAngle =
        Rotation2d.fromRadians(Math.atan((sourceHeight - pivotTranslation.getZ()) / distance2d));

    return Optional.of(new ShooterState(pivotToTargetAngle, SHOOTER_INTAKE_VELOCITY));
  }

  /**
   * Calculates the required angle of the to make a shot given a translation and a velocity vector
   *
   * @param translation translation representing the position of the robot
   * @param speeds velocity vector representing the current Vx, Vy, and Vtheta of the robot
   * @return required angle to make the shot
   */
  public static Rotation2d calculateRobotSpeakerAngle(
      Translation2d translation, ChassisSpeeds speeds) {
    // Due to projectile velocity, it isn't necessary to account for translational velocity vector
    // when determining shooting angle

    var targetTranslation = FieldConstants.Speaker.centerSpeaker.get().toTranslation2d();
    var robotToTargetTranslation =
        new Transform2d(GeomUtil.toPose2d(translation), GeomUtil.toPose2d(targetTranslation))
            .getTranslation();

    // Shooter faces the back, flip the angle
    return robotToTargetTranslation.getAngle().plus(Rotation2d.fromRadians(Math.PI));
  }

  public static Rotation2d calculateRobotIntakeAngle(Translation2d translation) {
    return FieldConstants.Source.SOURCE_WALL_ANGLE.get();
  }

  /**
   * Check if a position is somewhere we could shoot from
   *
   * @param robotPose pose to check
   * @return if we can shoot from that position
   */
  public static boolean inShooterZone(Pose2d robotPose) {
    return calculatePivotTranslation(robotPose)
            .toTranslation2d()
            .getDistance(FieldConstants.Speaker.centerSpeaker.get().toTranslation2d())
        <= SHOOTING_RADIUS_METERS;
  }

  /**
   * Check if a position is somewhere we could intake from
   *
   * @param robotPose pose to check
   * @return if we can intake from that position
   */
  public static boolean inIntakeZone(Pose2d robotPose) {
    return calculatePivotTranslation(robotPose)
            .toTranslation2d()
            .getDistance(FieldConstants.Source.SOURCE_MIDDLE_OPENING.get().toTranslation2d())
        <= INTAKE_RADIUS_METERS;
  }

  public static boolean withinSpeakerYawTolerance(Pose2d robotPose, ChassisSpeeds speeds) {
    var currentAngle = robotPose.getRotation();
    var requiredRobotAngle = calculateRobotSpeakerAngle(robotPose.getTranslation(), speeds);
    return Math.abs(requiredRobotAngle.getDegrees() - currentAngle.getDegrees())
        <= SHOOTER_SPEAKER_YAW_ANGLE_TOLERANCE_DEGREES;
  }
}
