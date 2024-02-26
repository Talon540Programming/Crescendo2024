package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  class RollerIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
