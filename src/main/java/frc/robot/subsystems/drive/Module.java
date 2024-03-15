package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber driveKs = new LoggedTunableNumber("DriveKs");
  private static final LoggedTunableNumber driveKv = new LoggedTunableNumber("DriveKv");
  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("DriveKp");
  private static final LoggedTunableNumber driveKi = new LoggedTunableNumber("DriveKi");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("DriveKd");

  private static final LoggedTunableNumber turnKp = new LoggedTunableNumber("TurnKp");
  private static final LoggedTunableNumber turnKi = new LoggedTunableNumber("TurnKi");
  private static final LoggedTunableNumber turnKd = new LoggedTunableNumber("TurnKd");

  static {
    // Drive PID is volts per RPM
    // Drive FF is volts per Rad/sec
    // Turn PID is volts per rotation of the motor shaft
    switch (Constants.getRobotType()) {
      case ROBOT_2024_COMP -> {
        driveKs.initDefault(0.2740042899903624);
        driveKv.initDefault(0.13424862447874678);
        driveKp.initDefault(0.0025);
        driveKi.initDefault(0.0);
        driveKd.initDefault(0.0);
        turnKp.initDefault(1.0);
        turnKi.initDefault(0.0);
        turnKd.initDefault(0.0);
      }
      case ROBOT_SIMBOT -> {
        driveKs.initDefault(0.06769487824001116);
        driveKv.initDefault(0.1338996940509799);
        driveKp.initDefault(1.5);
        driveKi.initDefault(0.0);
        driveKd.initDefault(0.0);
        turnKp.initDefault(5.5);
        turnKi.initDefault(0.0);
        turnKd.initDefault(0.0);
      }
    }
  }

  private final int moduleIndex;
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();

  // Calculates the drive motor's feedforward value in volts/radians per second
  private SimpleMotorFeedforward m_driveFeedforward;

  public Module(int index, ModuleIO io) {
    moduleIndex = index;
    m_io = io;
  }

  public void updateInputs() {
    m_io.updateInputs(m_inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> m_io.setDrivePID(driveKp.get(), driveKi.get(), driveKd.get()),
        driveKp,
        driveKi,
        driveKd);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> this.m_driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get()),
        driveKs,
        driveKv);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> m_io.setTurnPID(turnKp.get(), turnKi.get(), turnKd.get()),
        turnKp,
        turnKi,
        turnKd);
  }

  public void processInputs() {
    Logger.processInputs("Drive/Module" + moduleIndex, m_inputs);
  }

  public Rotation2d getAngle() {
    return m_inputs.turnPosition;
  }

  public double getDrivePositionRad() {
    return m_inputs.drivePositionRad;
  }

  public double getDriveVelocityRadPerSec() {
    return m_inputs.driveVelocityRadPerSec;
  }

  public double getDrivePositionMeters() {
    return getDrivePositionRad() * DriveBase.kWheelRadiusMeters;
  }

  public double getDriveVelocityMetersPerSec() {
    return getDriveVelocityRadPerSec() * DriveBase.kWheelRadiusMeters;
  }

  public SwerveModulePosition getCurrentPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), m_inputs.turnPosition);
  }

  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSec(), m_inputs.turnPosition);
  }

  public SwerveModulePosition[] getModuleDeltas() {
    return m_inputs.odometryPositions;
  }

  public SwerveModuleState runSetpoint(SwerveModuleState setpoint) {
    var optimizedSetpoint = SwerveModuleState.optimize(setpoint, getAngle());

    // TODO handle cos position error scaling
    double setpointVelocityRadPerSec =
        optimizedSetpoint.speedMetersPerSecond / DriveBase.kWheelRadiusMeters;
    m_io.runDriveVelocitySetpoint(
        setpointVelocityRadPerSec, m_driveFeedforward.calculate(setpointVelocityRadPerSec));
    m_io.runTurnAngleSetpoint(optimizedSetpoint.angle);

    return optimizedSetpoint;
  }

  public void runCharacterizationVoltage(Rotation2d angleSetpoint, double voltage) {
    m_io.runTurnAngleSetpoint(angleSetpoint);
    m_io.runDriveVolts(voltage);
  }

  public double getCharacterizationVelocity() {
    return getDriveVelocityRadPerSec();
  }

  public void setBrakeMode(boolean enabled) {
    m_io.setDriveBrakeMode(true);
    m_io.setTurnBrakeMode(true);
  }

  public void stop() {
    m_io.runDriveVolts(0);
    m_io.runTurnVolts(0);
  }
}
