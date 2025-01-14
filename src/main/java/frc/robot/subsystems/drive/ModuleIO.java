package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
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
