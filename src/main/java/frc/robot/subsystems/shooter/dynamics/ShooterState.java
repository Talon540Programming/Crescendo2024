package frc.robot.subsystems.shooter.dynamics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;

/**
 * Represent the state of the shooter at a given time
 *
 * @param angle the angle of the shooter relative to the plane intersecting the erector parallel to
 *     the ground.
 * @param shooterTopVelocityMetersPerSecond the velocity of the shooter's top flywheels in meters
 *     per second. Positive values represent shooting the note out while negative values draw notes
 *     in.
 * @param shooterBottomVelocityMetersPerSecond the velocity of the shooter's bottom flywheels in
 *     meters per second. Positive values represent shooting the note out while negative values draw
 *     notes in.
 */
public record ShooterState(
    Rotation2d angle,
    double shooterTopVelocityMetersPerSecond,
    double shooterBottomVelocityMetersPerSecond)
    implements StructSerializable {
  public static final ShooterState STARTING_STATE = new ShooterState(Rotation2d.fromDegrees(70), 0);
  public static final ShooterState TRAVEL_STATE = new ShooterState(Rotation2d.fromDegrees(35), 0);
  public static final ShooterState FEEDER_STATION_INTAKE =
      new ShooterState(Rotation2d.fromDegrees(71.740899), -10);

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
        velocityTolerance.initDefault(0.7);
        angleTolerance.initDefault(0.1);
      }
    }
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
    this(
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(),
                Constants.Shooter.MIN_SHOOTER_ANGLE.getRadians(),
                Constants.Shooter.MAX_SHOOTER_ANGLE.getRadians())),
        shooterVelocityMetersPerSecond,
        shooterVelocityMetersPerSecond);
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

  public static final ShooterStateStruct struct = new ShooterStateStruct();
}
