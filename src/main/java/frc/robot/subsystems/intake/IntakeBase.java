package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SingleJointedMechanismVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeBase extends SubsystemBase {
  public static final double WRIST_GEARING = (36.0 / 1.0);
  public static final double ROLLER_GEARING = (5.0 / 1.0);
  public static final double INDEXER_GEARING = (5.0 / 1.0);

  private final IndexerIO m_indexerIO;
  private final IndexerIOInputsAutoLogged m_indexerInputs = new IndexerIOInputsAutoLogged();

  private final WristIO m_wristIO;
  private final WristIOInputsAutoLogged m_wristInputs = new WristIOInputsAutoLogged();

  private final RollerIO m_rollerIO;
  private final RollerIOInputsAutoLogged m_rollerInputs = new RollerIOInputsAutoLogged();

  private Rotation2d m_wristGoal = Constants.Intake.STOW_ANGLE;

  private final ProfiledPIDController m_wristFeedback =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private static final LoggedTunableNumber wristKp = new LoggedTunableNumber("WristKp");
  private static final LoggedTunableNumber wristKi = new LoggedTunableNumber("WristKi");
  private static final LoggedTunableNumber wristKd = new LoggedTunableNumber("WristKd");

  private static final LoggedTunableNumber wristMaxVelocity =
      new LoggedTunableNumber("WristMaxVelocity");
  private static final LoggedTunableNumber wristMaxAcceleration =
      new LoggedTunableNumber("WristMaxAcceleration");

  private static final LoggedTunableNumber wristTolerance =
      new LoggedTunableNumber("WristTolerance");

  private final SingleJointedMechanismVisualizer m_goalVisualizer =
      new SingleJointedMechanismVisualizer(
          "Intake",
          "Goal",
          Constants.Intake.INTAKE_LENGTH_METERS,
          Constants.Intake.PIVOT_POSE,
          Constants.Intake.GROUND_INTAKE_ANGLE,
          2,
          2);

  private final SingleJointedMechanismVisualizer m_measuredVisualizer =
      new SingleJointedMechanismVisualizer(
          "Intake",
          "Measured",
          Constants.Intake.INTAKE_LENGTH_METERS,
          Constants.Intake.PIVOT_POSE,
          Constants.Intake.GROUND_INTAKE_ANGLE,
          2,
          2);

  private final SysIdRoutine m_wristCharacterizationRoutine;

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_2024_COMP -> {
        wristKp.initDefault(5.5);
        wristKi.initDefault(0.0);
        wristKd.initDefault(0.0);
        wristMaxVelocity.initDefault(Math.toRadians(90));
        wristMaxAcceleration.initDefault(Math.toRadians(180));
        wristTolerance.initDefault(Math.toRadians(4));
      }
      case ROBOT_SIMBOT -> {
        wristKp.initDefault(0.0); // TODO
        wristKi.initDefault(0.0); // TODO
        wristKd.initDefault(0.0); // TODO
        wristMaxVelocity.initDefault(0.0); // TODO
        wristMaxAcceleration.initDefault(0.0); // TODO
        wristTolerance.initDefault(0.0); // TODO
      }
    }
  }

  public IntakeBase(WristIO wristIO, RollerIO rollerIO, IndexerIO indexerIO) {
    this.m_wristIO = wristIO;
    this.m_rollerIO = rollerIO;
    this.m_indexerIO = indexerIO;

    m_wristIO.setBrakeMode(true);

    m_wristCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/WristSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> m_wristIO.setVoltage(voltage.in(Volts)), null, this, "Wrist"));
  }

  @Override
  public void periodic() {
    m_wristIO.updateInputs(m_wristInputs);
    m_rollerIO.updateInputs(m_rollerInputs);
    m_indexerIO.updateInputs(m_indexerInputs);

    Logger.processInputs("Intake/Wrist", m_wristInputs);
    Logger.processInputs("Intake/Roller", m_rollerInputs);
    Logger.processInputs("Intake/Indexer", m_indexerInputs);

    if (wristKp.hasChanged(0) || wristKi.hasChanged(0) || wristKd.hasChanged(0)) {
      m_wristFeedback.setPID(wristKp.get(), wristKi.get(), wristKd.get());
    }

    if (wristMaxVelocity.hasChanged(0) || wristMaxAcceleration.hasChanged(0)) {
      m_wristFeedback.setConstraints(
          new TrapezoidProfile.Constraints(wristMaxVelocity.get(), wristMaxAcceleration.get()));
    }

    if (wristTolerance.hasChanged(0)) {
      m_wristFeedback.setTolerance(wristTolerance.get());
    }

    if (DriverStation.isDisabled()) {
      m_wristGoal = Constants.Intake.STOW_ANGLE;

      // Disable mechanism
      m_wristIO.setVoltage(0);
      m_rollerIO.setVoltage(0);
      m_indexerIO.setVoltage(0);

      // Reset controllers
      m_wristFeedback.reset(
          m_wristInputs.absoluteAngle.getRadians(), m_wristInputs.velocityRadPerSec);

    } else if (m_wristGoal != null) {
      double wristMeasurement = m_wristInputs.absoluteAngle.getRadians();
      double wristGoal = m_wristGoal.getRadians();

      m_wristIO.setVoltage(
          MathUtil.clamp(m_wristFeedback.calculate(wristMeasurement, wristGoal), -12, 12));
    }

    if (m_wristGoal != null) {
      m_goalVisualizer.update(Rotation2d.fromRadians(Math.PI).minus(m_wristGoal));
    }

    m_measuredVisualizer.update(Rotation2d.fromRadians(Math.PI).minus(getWristAngle()));
  }

  public Rotation2d getWristAngle() {
    return m_wristInputs.absoluteAngle;
  }

  @AutoLogOutput(key = "Intake/WristAngleGoal")
  public Rotation2d getWristGoal() {
    return m_wristGoal;
  }

  public void setWristGoal(Rotation2d setpoint) {
    m_wristGoal =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                setpoint.getRadians(),
                Constants.Intake.MIN_ANGLE.getRadians(),
                Constants.Intake.MAX_ANGLE.getRadians()));
  }

  @AutoLogOutput(key = "Intake/AtWristSetpoint")
  public boolean atWristGoal() {
    return m_wristFeedback.atGoal();
  }

  public void setRollersVoltage(double volts) {
    m_rollerIO.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  public void setIndexerVoltage(double volts) {
    m_indexerIO.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  public Command characterizeWristQuasistatic(SysIdRoutine.Direction direction) {
    return handleCharacterization().andThen(m_wristCharacterizationRoutine.quasistatic(direction));
  }

  public Command characterizeWristDynamic(SysIdRoutine.Direction direction) {
    return handleCharacterization().andThen(m_wristCharacterizationRoutine.dynamic(direction));
  }

  private Command handleCharacterization() {
    return Commands.runOnce(
        () -> {
          m_wristGoal = null;
          m_wristIO.setVoltage(0);
          m_indexerIO.setVoltage(0);
          m_rollerIO.setVoltage(0);
        },
        this);
  }
}
