package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.swerve.SwerveSetpointGenerator.ModuleLimits;
import lombok.Builder;

public class DriveConstants {
  static final double odometryFrequencyHz = Constants.getMode() == Constants.Mode.SIM ? 50 : 250;

  // Distance b/t centroid of carpet contact surface of module wheels
  static final double trackWidthX = Units.inchesToMeters(22.5);
  static final double trackWidthY = Units.inchesToMeters(22.5);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);

  // Drive maxima, constants
  public static final double maxLinearVelocityMetersPerSec = Units.feetToMeters(15.1);
  public static final double maxLinearAccelerationMetersPerSecSquared = Units.feetToMeters(75.0);
  public static final double maxAngularVelocityRadPerSec =
      maxLinearVelocityMetersPerSec / driveBaseRadius;
  public static final double maxAngularAccelerationRadPerSecSquared = 2.0 * Math.PI;

  /* Robot dimensions = chassis + bumpers */
  public static final double driveWidth = Units.inchesToMeters(29.0);
  public static final double bumperThickness = Units.inchesToMeters(3.0);
  public static final double robotWidth = driveWidth + (2 * bumperThickness);

  // Location of each wheel from center of chassis
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  // Wheel radius
  public static final double wheelRadius = Units.inchesToMeters(2.0);

  // Module limits
  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          maxLinearVelocityMetersPerSec,
          maxLinearAccelerationMetersPerSecSquared,
          Units.degreesToRadians(1080.0));

  // Module Gearing
  // If we do Kraken or Redux, then implement like 6328
  static final double mk4iDriveGearing = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
  static final double mk4iTurnGearing = 12.8;

  // Module configurations
  // This is not on 2024's code.  Why?
  static final ModuleConfig[] moduleConfigs = {
    // Front Left
    ModuleConfig.builder()
        .turnMotorId(2)
        .driveMotorId(3)
        .encoderChannel(0)
        .encoderOffset(
            Rotation2d.fromRadians(
                1.7182357115138978)) // .rotateBy(Rotation2d.kPi)) (may not be needed for Krakens
        // depending on encoder)
        // Why .rotateBy(Rotation2d.kPi)?
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // Front Right
    ModuleConfig.builder()
        .turnMotorId(4)
        .driveMotorId(5)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRadians(-1.4361935561244243)) // .rotateBy(Rotation2d.kPi))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // Back Left
    ModuleConfig.builder()
        .turnMotorId(6)
        .driveMotorId(7)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(0.9998617084472845)) // .rotateBy(Rotation2d.kPi))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // Back Right
    ModuleConfig.builder()
        .turnMotorId(8)
        .driveMotorId(9)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(-1.7285578862473199)) // .rotateBy(Rotation2d.kPi))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
  };

  // Gyro/IMU ID Assignment
  static class PigeonConstants {
    public static final int id = 10;
  }

  @Builder
  record ModuleConfig(
      int turnMotorId,
      int driveMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      double driveGearing,
      double turnGearing,
      boolean turnInverted) {}
}
