package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP");
  private static final LoggedTunableNumber drivekI =
      new LoggedTunableNumber("Drive/Module/DrivekI");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD");
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS");
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV");
  private static final LoggedTunableNumber turnkP = new LoggedTunableNumber("Drive/Module/TurnkP");
  private static final LoggedTunableNumber turnkD = new LoggedTunableNumber("Drive/Module/TurnkD");

  //   private static final LoggedTunableNumber driveIZone = new
  // LoggedTunableNumber("Drive/Module/DrivekIZone");

  static {
    switch (Constants.getRobot()) {
      case OUTREACHBOT -> {
        drivekP.initDefault(0.0);
        drivekI.initDefault(0.0);
        drivekD.initDefault(0.0);
        drivekS.initDefault(0.0);
        drivekV.initDefault(0.0);
        turnkP.initDefault(0.0);
        turnkD.initDefault(0.0);
      }
      case SIMBOT -> {
        drivekP.initDefault(0.1);
        drivekI.initDefault(0.0);
        drivekD.initDefault(0.0);
        drivekS.initDefault(0.0);
        drivekV.initDefault(0.13);
        turnkP.initDefault(10.0);
        turnkD.initDefault(0.0);
      }
    }
  }

  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs =
      new ModuleIOInputsAutoLogged(); // Where is this?
  private final int index;

  private SimpleMotorFeedforward driveFeedforward;

  // Why is debouncer needed for these motors?
  // Connected debouncers
  private final Debouncer driveMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  // private final Debouncer turnEncoderConnectedDebouncer = new Debouncer(0.5,
  // Debouncer.DebounceType.kFalling);

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  // private final Alert turnEncoderDisconnectedAlert;

  @Getter private SwerveModulePosition[] odometryPositions; // = new SwerveModulePosition[] {}

  public Module(ModuleIO io, int index) {
    m_io = io;
    this.index = index;

    driveFeedforward = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());

    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
    // turnEncoderDisconnectedAlert = new Alert("Disconnected turn encoder on module " + index +
    // ".", AlertType.kError);
  }

  public void updateInputs() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Drive/Module" + index, m_inputs);
  }

  public void periodic() {
    // Update tunable numbers
    if (drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
      driveFeedforward = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
    }
    if (drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
      m_io.setDrivePID(drivekP.get(), 0, drivekD.get());
    }
    if (turnkP.hasChanged(hashCode()) || turnkD.hasChanged(hashCode())) {
      m_io.setTurnPID(turnkP.get(), 0, turnkD.get());
    }

    // Update Odometry Positions
    int sampleCount = m_inputs.odometryDrivePositionsRad.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = m_inputs.odometryDrivePositionsRad[i] * DriveConstants.wheelRadius;
      Rotation2d angle = m_inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update Alerts
    driveDisconnectedAlert.set(!m_inputs.driveConnected);
    turnDisconnectedAlert.set(!m_inputs.turnConnected);
    // turnEncoderDisconnectedAlert.set(!m_inputs.turnEncoderConnected) //CAN Implementation
    // (CANcoder or Redux Encoder)
    /* 6328 code */
    // driveDisconnectedAlert.set(
    //     !driveMotorConnectedDebouncer.calculate(inputs.data.driveConnected()) &&
    // !Robot.isJITing());
    // turnDisconnectedAlert.set(
    //     !turnMotorConnectedDebouncer.calculate(inputs.data.turnConnected()) &&
    // !Robot.isJITing());
    // turnEncoderDisconnectedAlert.set(
    //     !turnEncoderConnectedDebouncer.calculate(inputs.data.turnEncoderConnected()) &&
    // !Robot.isJITing());

    // Record cycle time
    LoggedTracer.record("Drive/Module" + index);
  }

  /** Runs the module with the specified setpoint state. */
  public void runSetpoint(SwerveModuleState state) {
    m_io.runDriveVelocity(state.speedMetersPerSecond / DriveConstants.wheelRadius);
    m_io.runTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    m_io.runDriveOpenLoop(output);
    m_io.runTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    m_io.runDriveOpenLoop(0.0);
    m_io.runTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return m_inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * DriveConstants.wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return m_inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return m_inputs.driveVelocityRadPerSec;
  }

  /* Sets brake mode to {@code enabled} */
  public void setDriveBrakeMode(boolean enabled) {
    m_io.setDriveBrakeMode(enabled);
  }
}
