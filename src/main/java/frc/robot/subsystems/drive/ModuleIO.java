package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.TimestampedSensorMeasurement;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {
  public static class ModuleIOInputs implements LoggableInputs, Cloneable {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public List<TimestampedSensorMeasurement<Double>> odometryDrivePositionsRad = List.of();
    public List<TimestampedSensorMeasurement<Rotation2d>> odometryTurnPositions = List.of();

    @Override
    public void toLog(LogTable table) {
      table.put("DrivePositionRad", drivePositionRad);
      table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
      table.put("DriveAppliedVolts", driveAppliedVolts);
      table.put("DriveCurrentAmps", driveCurrentAmps);
      table.put("TurnAbsolutePosition", turnAbsolutePosition);
      table.put("TurnPosition", turnPosition);
      table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
      table.put("TurnAppliedVolts", turnAppliedVolts);
      table.put("TurnCurrentAmps", turnCurrentAmps);

      // Separate sensor measurement pairs into discrete components
      table.put(
          "OdometryDrivePositionsRad",
          odometryDrivePositionsRad.stream()
              .mapToDouble(TimestampedSensorMeasurement::getMeasurement)
              .toArray());
      table.put(
          "OdometryDrivePositionsRadTimestamps",
          odometryDrivePositionsRad.stream()
              .mapToDouble(TimestampedSensorMeasurement::getTimestampSeconds)
              .toArray());
      table.put(
          "OdometryTurnPositions",
          odometryTurnPositions.stream()
              .map(TimestampedSensorMeasurement::getMeasurement)
              .toArray(Rotation2d[]::new));
      table.put(
          "OdometryTurnPositionsTimestamps",
          odometryTurnPositions.stream()
              .mapToDouble(TimestampedSensorMeasurement::getTimestampSeconds)
              .toArray());
    }

    @Override
    public void fromLog(LogTable table) {
      drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
      driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
      driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
      driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
      turnAbsolutePosition = table.get("TurnAbsolutePosition", turnAbsolutePosition);
      turnPosition = table.get("TurnPosition", turnPosition);
      turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", turnVelocityRadPerSec);
      turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
      turnCurrentAmps = table.get("TurnCurrentAmps", turnCurrentAmps);

      var currentDriveMeasurements =
          odometryDrivePositionsRad.stream()
              .mapToDouble(TimestampedSensorMeasurement::getMeasurement)
              .toArray();
      var currentDriveTimestamps =
          odometryDrivePositionsRad.stream()
              .mapToDouble(TimestampedSensorMeasurement::getTimestampSeconds)
              .toArray();
      var currentTurnMeasurements =
          odometryTurnPositions.stream()
              .map(TimestampedSensorMeasurement::getMeasurement)
              .toArray(Rotation2d[]::new);
      var currentTurnTimestamps =
          odometryTurnPositions.stream()
              .mapToDouble(TimestampedSensorMeasurement::getTimestampSeconds)
              .toArray();

      currentDriveMeasurements = table.get("OdometryDrivePositionsRad", currentDriveMeasurements);
      currentDriveTimestamps =
          table.get("OdometryDrivePositionsRadTimestamps", currentDriveTimestamps);
      currentTurnMeasurements = table.get("OdometryTurnPositions", currentTurnMeasurements);
      currentTurnTimestamps = table.get("OdometryTurnPositionsTimestamps", currentTurnTimestamps);

      odometryDrivePositionsRad = new ArrayList<>(currentDriveMeasurements.length);
      for (int i = 0; i < currentDriveMeasurements.length; i++) {
        odometryDrivePositionsRad.add(
            new TimestampedSensorMeasurement<>(
                currentDriveTimestamps[i], currentDriveMeasurements[i]));
      }
      odometryTurnPositions = new ArrayList<>(currentTurnMeasurements.length);
      for (int i = 0; i < currentTurnMeasurements.length; i++) {
        odometryTurnPositions.add(
            new TimestampedSensorMeasurement<>(
                currentTurnTimestamps[i], currentTurnMeasurements[i]));
      }
    }

    public ModuleIOInputs clone() {
      ModuleIOInputs copy = new ModuleIOInputs();
      copy.drivePositionRad = this.drivePositionRad;
      copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
      copy.driveAppliedVolts = this.driveAppliedVolts;
      copy.driveCurrentAmps = this.driveCurrentAmps.clone();
      copy.turnAbsolutePosition = this.turnAbsolutePosition;
      copy.turnPosition = this.turnPosition;
      copy.turnVelocityRadPerSec = this.turnVelocityRadPerSec;
      copy.turnAppliedVolts = this.turnAppliedVolts;
      copy.turnCurrentAmps = this.turnCurrentAmps.clone();
      copy.odometryDrivePositionsRad = List.copyOf(this.odometryDrivePositionsRad);
      copy.odometryTurnPositions = List.copyOf(this.odometryTurnPositions);
      return copy;
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
