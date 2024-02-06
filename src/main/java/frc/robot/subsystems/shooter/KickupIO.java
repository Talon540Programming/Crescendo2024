package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface KickupIO {
  @AutoLog
  class KickupIOInputs {
    public boolean forwardBeamBreakBroken;
    public boolean rearBeamBreakBroken;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(KickupIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
