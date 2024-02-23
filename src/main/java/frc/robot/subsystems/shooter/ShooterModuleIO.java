package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterModuleIO {
  @AutoLog
  class ShooterModuleIOInputs {
    public double velocityRadPerSec;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ShooterModuleIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
