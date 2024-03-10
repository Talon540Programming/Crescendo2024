package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.TimestampedSensorMeasurement;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
  public static class GyroIOInputs implements LoggableInputs, Cloneable {
    public boolean connected = false;

    public Rotation2d rollPosition = new Rotation2d();
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d yawPosition = new Rotation2d();

    public List<TimestampedSensorMeasurement<Rotation2d>> odometryYawPositions = List.of();

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
      table.put(
          "OdometryYawPositions",
          odometryYawPositions.stream()
              .map(TimestampedSensorMeasurement::getMeasurement)
              .toArray(Rotation2d[]::new));
      table.put(
          "OdometryYawPositionsTimestamps",
          odometryYawPositions.stream()
              .mapToDouble(TimestampedSensorMeasurement::getTimestampSeconds)
              .toArray());
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

      var currentMeasurements =
          odometryYawPositions.stream()
              .map(TimestampedSensorMeasurement::getMeasurement)
              .toArray(Rotation2d[]::new);
      var currentTimestamps =
          odometryYawPositions.stream()
              .mapToDouble(TimestampedSensorMeasurement::getTimestampSeconds)
              .toArray();
      currentMeasurements = table.get("OdometryYawPositions", currentMeasurements);
      currentTimestamps = table.get("OdometryYawPositionsTimestamps", currentTimestamps);

      odometryYawPositions = new ArrayList<>(currentMeasurements.length);
      for (int i = 0; i < currentMeasurements.length; i++) {
        odometryYawPositions.add(
            new TimestampedSensorMeasurement<>(currentTimestamps[i], currentMeasurements[i]));
      }

      rollVelocityRadPerSec = table.get("RollVelocityRadPerSec", rollVelocityRadPerSec);
      pitchVelocityRadPerSec = table.get("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
      yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
    }

    public GyroIOInputs clone() {
      GyroIOInputs copy = new GyroIOInputs();
      copy.connected = this.connected;
      copy.rollPosition = this.rollPosition;
      copy.pitchPosition = this.pitchPosition;
      copy.yawPosition = this.yawPosition;
      copy.odometryYawPositions = List.copyOf(this.odometryYawPositions);
      copy.rollVelocityRadPerSec = this.rollVelocityRadPerSec;
      copy.pitchVelocityRadPerSec = this.pitchVelocityRadPerSec;
      copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
      return copy;
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
