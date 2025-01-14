package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
  public static class GyroIOInputs implements LoggableInputs, Cloneable {
    public boolean connected = false;

    public Rotation2d rollPosition = new Rotation2d();
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d yawPosition = new Rotation2d();

    public double rollVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("RollPosition", rollPosition);
      table.put("PitchPosition", pitchPosition);
      table.put("YawPosition", yawPosition);
      // Separate sensor measurement pairs into discrete components

      table.put("RollVelocityRadPerSec", rollVelocityRadPerSec);
      table.put("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
      table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      rollPosition = table.get("RollPosition", rollPosition);
      pitchPosition = table.get("PitchPosition", pitchPosition);
      yawPosition = table.get("YawPosition", yawPosition);

      rollVelocityRadPerSec = table.get("RollVelocityRadPerSec", rollVelocityRadPerSec);
      pitchVelocityRadPerSec = table.get("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
      yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
