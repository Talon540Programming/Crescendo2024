package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean beamBreakBroken = false;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
