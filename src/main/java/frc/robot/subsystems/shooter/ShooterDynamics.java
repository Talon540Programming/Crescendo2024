package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterDynamics {
  public record ShooterState(
      Rotation2d angle,
      double shooterVelocityMetersPerSecond,
      double kickupVelocityMetersPerSecond) {
    public static ShooterState STARTING_STATE = new ShooterState(Rotation2d.fromDegrees(70), 0, 0);
    public static ShooterState TRAVEL_STATE = new ShooterState(Rotation2d.fromDegrees(35), 0, 0);

    @Override
    public boolean equals(Object obj) {
      if (obj instanceof ShooterState) {
        return angle.equals(((ShooterState) obj).angle)
            && Math.abs(
                    shooterVelocityMetersPerSecond
                        - ((ShooterState) obj).shooterVelocityMetersPerSecond)
                <= 1e-5
            && Math.abs(
                    kickupVelocityMetersPerSecond
                        - ((ShooterState) obj).kickupVelocityMetersPerSecond)
                <= 1e-5;
      }
      return false;
    }
  }
}
