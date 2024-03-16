package frc.robot.subsystems.shooter.dynamics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ShooterState implements StructSerializable {
  public static final ShooterState STARTING_STATE = new ShooterState(Rotation2d.fromDegrees(70), 0);
  public static final ShooterState TRAVEL_STATE = new ShooterState(Rotation2d.fromDegrees(25), 1.5);
  public static final ShooterState GROUND_INTAKE_STATE =
      new ShooterState(Rotation2d.fromDegrees(45), -1.5);

  public static final double SHOOTING_LOW_VELOCITY = 12.5;

  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Shooter/VelocityTolerance");
  private static final LoggedTunableNumber angleTolerance =
      new LoggedTunableNumber("Shooter/AngleTolerance");

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_SIMBOT -> {
        velocityTolerance.initDefault(0.01);
        angleTolerance.initDefault(0.01);
      }
      case ROBOT_2024_COMP -> {
        velocityTolerance.initDefault(1.5);
        angleTolerance.initDefault(0.1);
      }
    }
  }

  public final Rotation2d angle;
  public final double shooterTopVelocityMetersPerSecond;
  public final double shooterBottomVelocityMetersPerSecond;

  /**
   * Represent the state of the shooter at a given time
   *
   * @param angle the angle of the shooter relative to the plane intersecting the erector parallel
   *     to the ground.
   * @param shooterTopVelocityMetersPerSecond the velocity of the shooter's top flywheels in meters
   *     per second. Positive values represent shooting the note out while negative values draw
   *     notes in.
   * @param shooterBottomVelocityMetersPerSecond the velocity of the shooter's bottom flywheels in
   *     meters per second. Positive values represent shooting the note out while negative values
   *     draw notes in.
   */
  public ShooterState(
      Rotation2d angle,
      double shooterTopVelocityMetersPerSecond,
      double shooterBottomVelocityMetersPerSecond) {
    this.angle =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(),
                Constants.Shooter.MIN_SHOOTER_ANGLE.getRadians(),
                Constants.Shooter.MAX_SHOOTER_ANGLE.getRadians()));
    this.shooterTopVelocityMetersPerSecond =
        Math.copySign(
            Math.min(
                Math.abs(shooterTopVelocityMetersPerSecond),
                Constants.Shooter.MAX_SHOOTER_SPEED_METERS_PER_SECOND),
            shooterTopVelocityMetersPerSecond);
    this.shooterBottomVelocityMetersPerSecond =
        Math.copySign(
            Math.min(
                Math.abs(shooterBottomVelocityMetersPerSecond),
                Constants.Shooter.MAX_SHOOTER_SPEED_METERS_PER_SECOND),
            shooterBottomVelocityMetersPerSecond);
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
  public ShooterState(Rotation2d angle, double shooterVelocityMetersPerSecond) {
    this(angle, shooterVelocityMetersPerSecond, shooterVelocityMetersPerSecond);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ShooterState other) {
      return Math.hypot(
                  angle.getCos() - other.angle.getCos(), angle.getSin() - other.angle.getSin())
              < angleTolerance.get()
          && Math.abs(shooterTopVelocityMetersPerSecond - other.shooterTopVelocityMetersPerSecond)
              <= velocityTolerance.get()
          && Math.abs(
                  shooterBottomVelocityMetersPerSecond - other.shooterBottomVelocityMetersPerSecond)
              <= velocityTolerance.get();
    }
    return false;
  }

  // Required for struct serialization
  public static final ShooterStateStruct struct = new ShooterStateStruct();
}
