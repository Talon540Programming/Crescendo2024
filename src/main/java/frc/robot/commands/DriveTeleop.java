package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.oi.DriveOI;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseEstimator;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveTeleop extends DriveHeading {
  private static final LoggedTunableNumber controllerDeadband =
      new LoggedTunableNumber("TeleopDrive/Deadband", 0.1);
  private static final LoggedTunableNumber maxAngularVelocityScalar =
      new LoggedTunableNumber("TeleopDrive/AngularVelocityScalar", 0.75);

  private final DriveBase driveBase;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier thetaSupplier;
  private final BooleanSupplier robotRelativeSupplier;
  private final BooleanSupplier headingLockSupplier;

  public DriveTeleop(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier thetaSupplier,
      BooleanSupplier robotRelativeSupplier,
      BiFunction<Pose2d, ChassisSpeeds, Optional<Rotation2d>> headingSupplier,
      BooleanSupplier headingLockSupplier) {
    super(driveBase, headingSupplier);

    this.driveBase = driveBase;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.thetaSupplier = thetaSupplier;
    this.robotRelativeSupplier = robotRelativeSupplier;
    this.headingLockSupplier = headingLockSupplier;

    // Because we are extending the command, we need to replace the parent name
    setName("DriveTeleop");
  }

  public DriveTeleop(
      DriveBase driveBase,
      DriveOI oi,
      BiFunction<Pose2d, ChassisSpeeds, Optional<Rotation2d>> headingSupplier) {
    this(
        driveBase,
        oi::getDriveX,
        oi::getDriveY,
        oi::getDriveTheta,
        () -> oi.robotRelativeOverride().getAsBoolean(),
        headingSupplier,
        () -> oi.headingLock().getAsBoolean());
  }

  @Override
  public void execute() {
    // Because we aren't calling super.execute(), we need this
    super.pollTunableNumbers();

    Pose2d currentPose = PoseEstimator.getInstance().getPose();
    double deadband = controllerDeadband.get();
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), deadband);
    double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), deadband);
    double theta = MathUtil.applyDeadband(thetaSupplier.getAsDouble(), deadband);

    // Square values
    x = Math.copySign(Math.pow(x, 2), x);
    y = Math.copySign(Math.pow(y, 2), y);
    theta = Math.copySign(Math.pow(theta, 2), theta);

    // Scale angular velocity by scalar
    theta *= maxAngularVelocityScalar.get();

    ChassisSpeeds speeds;
    if (robotRelativeSupplier.getAsBoolean()) {
      speeds =
          new ChassisSpeeds(
              x * DriveBase.kMaxLinearVelocityMetersPerSecond,
              y * DriveBase.kMaxLinearVelocityMetersPerSecond,
              theta * DriveBase.kMaxAngularVelocityRadiansPerSecond);
    } else {
      var linearVelocity = new Translation2d(x, y);

      // // Flip the direction of the translational component for field relative based on alliance
      // // swap. Pose is always relative to blue alliance, so swap if red
      if (AllianceFlipUtil.shouldFlip()) {
        linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
      }

      // Rotate speed to account for field relative
      var fieldRelativeVelocity = linearVelocity.rotateBy(currentPose.getRotation().unaryMinus());

      speeds =
          new ChassisSpeeds(
              fieldRelativeVelocity.getX() * DriveBase.kMaxLinearVelocityMetersPerSecond,
              fieldRelativeVelocity.getY() * DriveBase.kMaxLinearVelocityMetersPerSecond,
              theta * DriveBase.kMaxAngularVelocityRadiansPerSecond);
    }

    // Replace omega velocity component with calculated speaker lock value using heading controller
    if (headingLockSupplier.getAsBoolean()) {
      calculateSpeeds().ifPresent((v) -> speeds.omegaRadiansPerSecond = v.omegaRadiansPerSecond);
    }

    driveBase.runVelocity(speeds);
  }
}
