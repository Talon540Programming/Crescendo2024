package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.PoseEstimator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriveCommandFactory {
  private static final LoggedDashboardChooser<Double> xCoefficient =
      new LoggedDashboardChooser<>("Drive X Speed Limiter");
  private static final LoggedDashboardChooser<Double> yCoefficient =
      new LoggedDashboardChooser<>("Drive Y Speed Limiter");
  private static final LoggedDashboardChooser<Double> omegaCoefficient =
      new LoggedDashboardChooser<>("Drive Omega Speed Limiter");

  static {
    xCoefficient.addDefaultOption("Default (100%)", 1.0);
    xCoefficient.addOption("Fast (70%)", 0.7);
    xCoefficient.addOption("Medium (30%)", 0.3);
    xCoefficient.addOption("Slow (15%)", 0.15);

    yCoefficient.addDefaultOption("Default (100%)", 1.0);
    yCoefficient.addOption("Fast (70%)", 0.7);
    yCoefficient.addOption("Medium (30%)", 0.3);
    yCoefficient.addOption("Slow (15%)", 0.15);

    omegaCoefficient.addDefaultOption("Default (100%)", 1.0);
    omegaCoefficient.addOption("Fast (70%)", 0.7);
    omegaCoefficient.addOption("Medium (30%)", 0.3);
    omegaCoefficient.addOption("Slow (15%)", 0.15);
  }

  public static Command joystickDrive(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double deadband) {
    if (deadband >= 1.0) {
      throw new IllegalArgumentException(
          String.format(
              "Invalid Deadband Configured. Deadband value should be (0.0, 1.0), supplied: %f",
              deadband));
    }

    return Commands.run(
        () -> {
          double x_val = MathUtil.applyDeadband(xSupplier.getAsDouble(), deadband);
          double y_val = MathUtil.applyDeadband(ySupplier.getAsDouble(), deadband);
          double omega_val = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadband);

          double x = 0.0;
          double y = 0.0;
          double omega = 0.0;

          if (x_val != 0) {
            x_val = x_val * xCoefficient.get();
            x = Math.copySign(Math.min(Math.pow(x_val, 2), 1.0), x_val);
          }

          if (y_val != 0) {
            y_val = y_val * yCoefficient.get();
            y = Math.copySign(Math.min(Math.pow(y_val, 2), 1.0), y_val);
          }

          if (omega_val != 0) {
            omega_val = omega_val * omegaCoefficient.get();
            omega = Math.copySign(Math.min(Math.pow(omega_val, 2), 1.0), omega_val);
          }

          driveBase.runVelocity(toFieldRelative(x, y, omega));
        },
        driveBase);
  }

  public static Command sprintJoystickDrive(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier sprintSupplier,
      double maxNonSprintSpeed,
      double deadband) {
    if (maxNonSprintSpeed >= 1.0) {
      throw new IllegalArgumentException(
          String.format(
              "Invalid non-sprint max speed. Max Sprint speed should be (0.0, 1.0), supplied %f",
              maxNonSprintSpeed));
    }
    if (deadband >= 1.0) {
      throw new IllegalArgumentException(
          String.format(
              "Invalid Deadband Configured. Deadband value should be (0.0, 1.0), supplied: %f",
              deadband));
    }

    return Commands.run(
        () -> {
          double x_val = MathUtil.applyDeadband(xSupplier.getAsDouble(), deadband);
          double y_val = MathUtil.applyDeadband(ySupplier.getAsDouble(), deadband);
          double omega_val = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadband);

          double x = 0.0;
          double y = 0.0;
          double omega = 0.0;

          double sprintIncreaseVal = 1.0 - maxNonSprintSpeed;

          if (x_val != 0) {
            x_val =
                (x_val * xCoefficient.get() * maxNonSprintSpeed)
                    + (sprintSupplier.getAsBoolean() ? sprintIncreaseVal : 0.0);
            x = Math.copySign(Math.min(Math.pow(x_val, 2), 1.0), x_val);
          }

          if (y_val != 0) {
            y_val =
                (y_val * yCoefficient.get() * maxNonSprintSpeed)
                    + (sprintSupplier.getAsBoolean() ? sprintIncreaseVal : 0.0);
            y = Math.copySign(Math.min(Math.pow(y_val, 2), 1.0), y_val);
          }

          if (omega_val != 0) {
            omega_val = omega_val * omegaCoefficient.get();
            omega = Math.copySign(Math.min(Math.pow(omega_val, 2), 1.0), omega_val);
          }

          driveBase.runVelocity(toFieldRelative(x, y, omega));
        },
        driveBase);
  }

  private static ChassisSpeeds toFieldRelative(double x, double y, double omega) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        x * Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
        y * Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
        omega * Constants.Drivetrain.kMaxAngularVelocityRadiansPerSecond,
        PoseEstimator.getInstance().getPose().getRotation());
  }

  private static ChassisSpeeds toRobotRelative(double x, double y, double omega) {
    return new ChassisSpeeds(
        x * Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
        y * Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
        omega * Constants.Drivetrain.kMaxAngularVelocityRadiansPerSecond);
  }
}
