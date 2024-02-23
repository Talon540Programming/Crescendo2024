package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ErectorIO {
  @AutoLog
  class ErectorIOInputs {
    public double velocityRadPerSec = 0.0;
    public Rotation2d absoluteAngle = new Rotation2d();
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ErectorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setBrakeMode(boolean enable) {}
}
