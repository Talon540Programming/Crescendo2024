package frc.robot.subsystems.shooter.dynamics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.StructSerializable;

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
              < 5e-3
          && Math.abs(shooterTopVelocityMetersPerSecond - other.shooterTopVelocityMetersPerSecond)
              <= 0.5
          && Math.abs(
                  shooterBottomVelocityMetersPerSecond - other.shooterBottomVelocityMetersPerSecond)
              <= 0.5;
    }
    return false;
  }

  public static final ShooterStateStruct struct = new ShooterStateStruct();
}
