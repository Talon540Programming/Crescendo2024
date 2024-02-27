package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterModuleIO {
  @AutoLog
  class ShooterModuleIOInputs {
    public double topPositionRad = 0.0;
    public double topVelocityRadPerSecond = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;

    public double bottomPositionRad = 0.0;
    public double bottomVelocityRadPerSecond = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterModuleIOInputs inputs) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void runSetpoint(
      double topSetpointRadPerSec,
      double bottomSetpointRadPerSec,
      double topFeedforwardVoltage,
      double bottomFeedforwardVoltage) {}

  public default void runCharacterizationVoltage(double voltage) {}

  public default void stop() {}
}
