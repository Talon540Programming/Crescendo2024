package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void runDriveVolts(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void runTurnVolts(double volts) {}

  /** Set the drive PID coefficients used */
  public default void setDrivePID(double kP, double kI, double kD) {}

  /** Set the turn PID coefficients used */
  public default void setTurnPID(double kP, double kI, double kD) {}

  /** Set the setpoint of the drive PID controller with an arbitrary feedforward value in volts */
  public default void runDriveVelocitySetpoint(
      double velocityRadPerSec, double driveFeedForwardVolts) {}

  /** Set the setpoint of the turn PID controller */
  public default void runTurnAngleSetpoint(Rotation2d angle) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
