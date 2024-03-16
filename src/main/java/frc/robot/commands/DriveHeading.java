package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseEstimator;
import java.util.Optional;
import java.util.function.BiFunction;

public class DriveHeading extends Command {
  private static final LoggedTunableNumber headingKp =
      new LoggedTunableNumber("DriveHeading/HeadingKp");
  private static final LoggedTunableNumber headingKd =
      new LoggedTunableNumber("DriveHeading/HeadingKd");
  private static final LoggedTunableNumber headingToleranceDegrees =
      new LoggedTunableNumber("DriveHeading/HeadingToleranceDegrees");

  private static final LoggedTunableNumber headingMaxVelocityScalar =
      new LoggedTunableNumber("DriveHeading/HeadingMaxVelocityScalar");
  private static final LoggedTunableNumber headingMaxAccelerationScalar =
      new LoggedTunableNumber("DriveHeading/HeadingMaxAccelerationScalar");

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
  private final BiFunction<Pose2d, ChassisSpeeds, Optional<Rotation2d>> headingSupplier;
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  public DriveHeading(
      DriveBase driveBase,
      BiFunction<Pose2d, ChassisSpeeds, Optional<Rotation2d>> headingSupplier) {
    addRequirements(driveBase);
    this.driveBase = driveBase;
    this.headingSupplier = headingSupplier;

    this.headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  protected Optional<ChassisSpeeds> calculateSpeeds() {
    var currentPose = PoseEstimator.getInstance().getPose();
    var currentSpeeds = driveBase.getVelocity();
    return headingSupplier
        .apply(currentPose, currentSpeeds)
        .map(
            v ->
                // We only want to rotate in place so ignore Vx and Vy
                new ChassisSpeeds(
                    0,
                    0,
                    headingController.calculate(
                        currentPose.getRotation().getRadians(), v.getRadians())));
  }

  protected void pollTunableNumbers() {
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
    pollTunableNumbers();
    calculateSpeeds().ifPresent(driveBase::runVelocity);
  }
}
