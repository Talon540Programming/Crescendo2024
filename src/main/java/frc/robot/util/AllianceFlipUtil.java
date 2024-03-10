package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;
import java.util.function.Function;

public class AllianceFlipUtil {
  private static double flipX(double x) {
    return FieldConstants.FIELD_LAYOUT.getFieldWidth() - x;
  }

  private static Translation2d flipTranslation(Translation2d translation2d) {
    return new Translation2d(flipX(translation2d.getX()), translation2d.getY());
  }

  private static Translation3d flipTranslation(Translation3d translation3d) {
    return new Translation3d(
        flipX(translation3d.getX()), translation3d.getY(), translation3d.getZ());
  }

  private static Rotation2d flipRotation(Rotation2d rotation) {
    return new Rotation2d(-rotation.getCos(), rotation.getSin());
  }

  private static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
  }

  private static boolean shouldFlip() {
    var currentAllianceOpt = DriverStation.getAlliance();
    return currentAllianceOpt.isPresent() && currentAllianceOpt.get() == DriverStation.Alliance.Red;
  }

  public static double apply(double x) {
    return shouldFlip() ? flipX(x) : x;
  }

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip() ? flipTranslation(translation) : translation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? flipRotation(rotation) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? flipPose(pose) : pose;
  }

  public static Translation3d apply(Translation3d translation3d) {
    return shouldFlip() ? flipTranslation(translation3d) : translation3d;
  }

  public static class AllianceRelative<T> {
    private final T value;
    private final Function<T, T> transformer;

    public AllianceRelative(T value, Function<T, T> transformer) {
      this.value = value;
      this.transformer = transformer;
    }

    public T get() {
      return shouldFlip() ? transformer.apply(value) : value;
    }

    public static AllianceRelative<Rotation2d> from(Rotation2d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::flipRotation);
    }

    public static AllianceRelative<Translation2d> from(Translation2d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::flipTranslation);
    }

    public static AllianceRelative<Pose2d> from(Pose2d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::flipPose);
    }

    public static AllianceRelative<Translation3d> from(Translation3d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::flipTranslation);
    }
  }
}
