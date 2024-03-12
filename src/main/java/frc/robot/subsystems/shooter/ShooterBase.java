package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.dynamics.ShooterState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SingleJointedMechanismVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterBase extends SubsystemBase {
  public static final double ERECTOR_GEARING =
      (52.0 / 12.0) * (52.0 / 18.0) * (52.0 / 18.0) * (58.0 / 24.0);
  public static final double SHOOTER_GEARING = (26.0 / 51.0);
  public static final double SHOOTER_RADIUS_METERS = Units.inchesToMeters(1.5);
  public static final double KICKUP_GEARING = (5.0 / 1.0);

  private static final LoggedTunableNumber erectorKs = new LoggedTunableNumber("ErectorKs");
  private static final LoggedTunableNumber erectorKg = new LoggedTunableNumber("ErectorKg");
  private static final LoggedTunableNumber erectorKv = new LoggedTunableNumber("ErectorKv");
  private static final LoggedTunableNumber erectorKa = new LoggedTunableNumber("ErectorKa");

  private static final LoggedTunableNumber erectorKp = new LoggedTunableNumber("ErectorKp");
  private static final LoggedTunableNumber erectorKi = new LoggedTunableNumber("ErectorKi");
  private static final LoggedTunableNumber erectorKd = new LoggedTunableNumber("ErectorKd");

  private static final LoggedTunableNumber erectorMaxVelocity =
      new LoggedTunableNumber("ErectorMaxVelocity");
  private static final LoggedTunableNumber erectorMaxAcceleration =
      new LoggedTunableNumber("ErectorMaxAcceleration");

  private static final LoggedTunableNumber shooterModuleKs =
      new LoggedTunableNumber("ShooterModuleKs");
  private static final LoggedTunableNumber shooterModuleKv =
      new LoggedTunableNumber("ShooterModuleKv");

  private static final LoggedTunableNumber shooterModuleKp =
      new LoggedTunableNumber("ShooterModuleKp");
  private static final LoggedTunableNumber shooterModuleKi =
      new LoggedTunableNumber("ShooterModuleKi");
  private static final LoggedTunableNumber shooterModuleKd =
      new LoggedTunableNumber("ShooterModuleKd");

  static {
    switch (Constants.getRobotType()) {
        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A19.3879039%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A10.399172%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A87.3978%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-215%2C%22u%22%3A%22deg%22%7D
      case ROBOT_2024_COMP -> {
        erectorKs.initDefault(0.0);
        erectorKg.initDefault(0.485);
        erectorKv.initDefault(0.0);
        erectorKa.initDefault(0.0);
        erectorKp.initDefault(5.5);
        erectorKi.initDefault(0.0);
        erectorKd.initDefault(0.0);

        erectorMaxVelocity.initDefault(Math.PI);
        erectorMaxAcceleration.initDefault(1.5 * Math.PI);

        shooterModuleKs.initDefault(0.5);
        shooterModuleKv.initDefault(0.0094048);
        shooterModuleKp.initDefault(0.0001);
        shooterModuleKi.initDefault(0.0);
        shooterModuleKd.initDefault(0.0);
      }
      case ROBOT_SIMBOT -> {
        erectorKs.initDefault(0.0); // TODO
        erectorKg.initDefault(0.0); // TODO
        erectorKv.initDefault(0.0); // TODO
        erectorKa.initDefault(0.0); // TODO
        erectorKp.initDefault(0.0); // TODO
        erectorKi.initDefault(0.0); // TODO
        erectorKd.initDefault(0.0); // TODO
        erectorMaxVelocity.initDefault(0.0); // TODO
        erectorMaxAcceleration.initDefault(0.0); // TODO
        shooterModuleKs.initDefault(0.0); // TODO
        shooterModuleKv.initDefault(0.0); // TODO
        shooterModuleKp.initDefault(0.0); // TODO
        shooterModuleKi.initDefault(0.0); // TODO
        shooterModuleKd.initDefault(0.0); // TODO
      }
    }
  }

  private final ErectorIO m_erectorIO;
  private final ErectorIOInputsAutoLogged m_erectorInputs = new ErectorIOInputsAutoLogged();

  private final ShooterModuleIO m_shooterModuleIO;
  private final ShooterModuleIOInputsAutoLogged m_shooterModuleInputs =
      new ShooterModuleIOInputsAutoLogged();

  private final KickupIO m_kickupIO;
  private final KickupIOInputsAutoLogged m_kickupInputs = new KickupIOInputsAutoLogged();

  private ShooterState m_setpoint = ShooterState.TRAVEL_STATE;

  private ArmFeedforward m_erectorFeedforward = new ArmFeedforward(0, 0, 0);
  private final ProfiledPIDController m_erectorFeedback =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private SimpleMotorFeedforward m_shooterModuleFeedforward = new SimpleMotorFeedforward(0, 0);

  private final SingleJointedMechanismVisualizer m_setpointVisualizer =
      new SingleJointedMechanismVisualizer(
          "Shooter",
          "Setpoint",
          Constants.Shooter.SHOOTER_LENGTH_METERS,
          Constants.Shooter.PIVOT_POSE,
          ShooterState.STARTING_STATE.angle,
          2,
          2);
  private final SingleJointedMechanismVisualizer m_measuredVisualizer =
      new SingleJointedMechanismVisualizer(
          "Shooter",
          "Measured",
          Constants.Shooter.SHOOTER_LENGTH_METERS,
          Constants.Shooter.PIVOT_POSE,
          ShooterState.STARTING_STATE.angle,
          2,
          2);

  public ShooterBase(ErectorIO erectorIO, ShooterModuleIO shooterModuleIO, KickupIO kickupIO) {
    this.m_erectorIO = erectorIO;
    this.m_shooterModuleIO = shooterModuleIO;
    this.m_kickupIO = kickupIO;

    m_erectorIO.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    m_shooterModuleIO.updateInputs(m_shooterModuleInputs);
    m_erectorIO.updateInputs(m_erectorInputs);
    m_kickupIO.updateInputs(m_kickupInputs);

    Logger.processInputs("Shooter/Erector", m_erectorInputs);
    Logger.processInputs("Shooter/Module", m_shooterModuleInputs);
    Logger.processInputs("Shooter/Kickup", m_kickupInputs);

    LoggedTunableNumber.ifChanged(
        () ->
            m_erectorFeedforward =
                new ArmFeedforward(
                    erectorKs.get(), erectorKg.get(), erectorKv.get(), erectorKa.get()),
        erectorKs,
        erectorKg,
        erectorKv,
        erectorKa);

    LoggedTunableNumber.ifChanged(
        () -> m_erectorFeedback.setPID(erectorKp.get(), erectorKi.get(), erectorKd.get()),
        erectorKp,
        erectorKi,
        erectorKd);

    LoggedTunableNumber.ifChanged(
        () ->
            m_erectorFeedback.setConstraints(
                new TrapezoidProfile.Constraints(
                    erectorMaxVelocity.get(), erectorMaxAcceleration.get())),
        erectorMaxVelocity,
        erectorMaxAcceleration);

    LoggedTunableNumber.ifChanged(
        () ->
            m_shooterModuleFeedforward =
                new SimpleMotorFeedforward(shooterModuleKs.get(), shooterModuleKv.get()),
        shooterModuleKs,
        shooterModuleKv);

    LoggedTunableNumber.ifChanged(
        () ->
            m_shooterModuleIO.setPID(
                shooterModuleKp.get(), shooterModuleKi.get(), shooterModuleKd.get()),
        shooterModuleKp,
        shooterModuleKi,
        shooterModuleKd);

    // Update setpoint and command IO layers
    if (DriverStation.isDisabled()) {
      // Reset visualizer to travel state when disabled
      m_setpoint = ShooterState.TRAVEL_STATE;

      // Disable the erector and shooter
      m_shooterModuleIO.stop();
      m_erectorIO.setVoltage(0.0);
      m_kickupIO.setVoltage(0.0);

      // Reset controllers
      m_erectorFeedback.reset(
          m_erectorInputs.absoluteAngle.getRadians(), m_erectorInputs.velocityRadPerSec);
    } else if (m_setpoint != null) {
      double erectorMeasurement = m_erectorInputs.absoluteAngle.getRadians();
      double erectorGoal = m_setpoint.angle.getRadians();

      m_erectorIO.setVoltage(
          MathUtil.clamp(
              m_erectorFeedforward.calculate(erectorGoal, 0.0)
                  + m_erectorFeedback.calculate(erectorMeasurement, erectorGoal),
              -12,
              12));

      double topSetpoint = m_setpoint.shooterTopVelocityMetersPerSecond / SHOOTER_RADIUS_METERS;
      double bottomSetpoint =
          m_setpoint.shooterBottomVelocityMetersPerSecond / SHOOTER_RADIUS_METERS;

      m_shooterModuleIO.runSetpoint(
          topSetpoint,
          bottomSetpoint,
          m_shooterModuleFeedforward.calculate(topSetpoint),
          m_shooterModuleFeedforward.calculate(bottomSetpoint));
    }

    if (m_setpoint != null) {
      m_setpointVisualizer.update(m_setpoint.angle);
    }
    m_measuredVisualizer.update(m_erectorInputs.absoluteAngle);
  }

  public void setSetpoint(ShooterState state) {
    m_setpoint = state;
  }

  @AutoLogOutput(key = "Shooter/Setpoint")
  public ShooterState getSetpoint() {
    return m_setpoint;
  }

  @AutoLogOutput(key = "Shooter/CurrentState")
  public ShooterState getCurrentState() {
    return new ShooterState(
        m_erectorInputs.absoluteAngle,
        m_shooterModuleInputs.topVelocityRadPerSecond * SHOOTER_RADIUS_METERS,
        m_shooterModuleInputs.bottomVelocityRadPerSecond * SHOOTER_RADIUS_METERS);
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return getSetpoint().equals(getCurrentState());
  }

  public void setKickupVoltage(double volts) {
    m_kickupIO.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  public boolean holdingNote() {
    return m_kickupInputs.beamBreakBroken;
  }
}
