package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.oi.DriveOI;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseEstimator;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveTeleop extends Command {
  private static final LoggedTunableNumber controllerDeadband =
      new LoggedTunableNumber("TeleopDrive/Deadband", 0.1);
  private static final LoggedTunableNumber maxAngularVelocityScalar =
      new LoggedTunableNumber("TeleopDrive/AngularVelocityScalar", 0.75);

  private static final LoggedTunableNumber headingKp =
      new LoggedTunableNumber("TeleopDrive/HeadingKp");
  private static final LoggedTunableNumber headingKd =
      new LoggedTunableNumber("TeleopDrive/HeadingKd");
  private static final LoggedTunableNumber headingToleranceDegrees =
      new LoggedTunableNumber("TeleopDrive/HeadingToleranceDegrees");

  private static final LoggedTunableNumber headingMaxVelocityScalar =
      new LoggedTunableNumber("TeleopDrive/HeadingMaxVelocityScalar");
  private static final LoggedTunableNumber headingMaxAccelerationScalar =
      new LoggedTunableNumber("TeleopDrive/HeadingMaxAccelerationScalar");

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_SIMBOT, ROBOT_2024_COMP -> {
        headingKp.initDefault(5.0);
        headingKd.initDefault(0.0);
        headingToleranceDegrees.initDefault(1.0);
        headingMaxVelocityScalar.initDefault(0.8);
        headingMaxAccelerationScalar.initDefault(0.8);
      }
    }
  }

  private final DriveBase driveBase;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier thetaSupplier;
  private final BooleanSupplier robotRelativeSupplier;
  private final BiFunction<Pose2d, ChassisSpeeds, Optional<Rotation2d>> headingSupplier;
  private final BooleanSupplier headingLockSupplier;

  private final ProfiledPIDController headingController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  public DriveTeleop(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier thetaSupplier,
      BooleanSupplier robotRelativeSupplier,
      BiFunction<Pose2d, ChassisSpeeds, Optional<Rotation2d>> headingSupplier,
      BooleanSupplier headingLockSupplier) {
    addRequirements(driveBase);

    this.driveBase = driveBase;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.thetaSupplier = thetaSupplier;
    this.robotRelativeSupplier = robotRelativeSupplier;
    this.headingSupplier = headingSupplier;
    this.headingLockSupplier = headingLockSupplier;

    headingController.enableContinuousInput(-Math.PI, Math.PI);
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
  public void initialize() {
    headingController.setPID(headingKp.get(), 0, headingKd.get());
    headingController.setConstraints(
        new TrapezoidProfile.Constraints(
            DriveBase.kMaxAngularVelocityRadiansPerSecond * headingMaxVelocityScalar.get(),
            DriveBase.kMaxAngularAccelerationRadiansPerSecondSquared
                * headingMaxAccelerationScalar.get()));
    headingController.setTolerance(Units.degreesToRadians(headingToleranceDegrees.get()));

    headingController.reset(
        PoseEstimator.getInstance().getPose().getRotation().getRadians(),
        driveBase.getVelocity().omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> headingController.setPID(headingKp.get(), 0, headingKd.get()),
        headingKp,
        headingKd);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            headingController.setConstraints(
                new TrapezoidProfile.Constraints(
                    DriveBase.kMaxAngularVelocityRadiansPerSecond * headingMaxVelocityScalar.get(),
                    DriveBase.kMaxAngularAccelerationRadiansPerSecondSquared
                        * headingMaxAccelerationScalar.get())),
        headingMaxVelocityScalar,
        headingMaxAccelerationScalar);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> headingController.setTolerance(Units.degreesToRadians(headingToleranceDegrees.get())),
        headingToleranceDegrees);

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
      var allianceOpt = DriverStation.getAlliance();
      if (allianceOpt.isPresent() && allianceOpt.get() == DriverStation.Alliance.Red) {
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
      headingSupplier
          .apply(currentPose, driveBase.getVelocity())
          .ifPresent(
              v -> {
                speeds.omegaRadiansPerSecond =
                    headingController.calculate(
                        currentPose.getRotation().getRadians(), v.getRadians());
                Logger.recordOutput(
                    "TeleopDrive/HeadingControllerPose",
                    new Pose2d(currentPose.getTranslation(), v));
                Logger.recordOutput(
                    "TeleopDrive/HeadingError", headingController.getPositionError());
              });
    }

    driveBase.runVelocity(speeds);
  }
}
