package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterDynamics.ShooterState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterBase extends SubsystemBase {
  public static final double ERECTOR_GEARING =
      (52.0 / 12.0) * (52.0 / 18.0) * (52.0 / 18.0) * (58.0 / 24.0);
  public static final double SHOOTER_GEARING = (26.0 / 52.0);
  public static final double SHOOTER_RADIUS_METERS = Units.inchesToMeters(1.5);
  public static final double KICKUP_GEARING = (5.0 / 1.0);
  public static final double KICKUP_RADIUS_METERS = Units.inchesToMeters(1.0);

  private final ErectorIO m_erectorIO;
  private final ErectorIOInputsAutoLogged m_erectorInputs = new ErectorIOInputsAutoLogged();

  private final ShooterModuleIO m_shooterModuleIO;
  private final ShooterModuleIOInputsAutoLogged m_shooterModuleInputs =
      new ShooterModuleIOInputsAutoLogged();

  private final KickupIO m_kickupIO;
  private final KickupIOInputsAutoLogged m_kickupInputs = new KickupIOInputsAutoLogged();

  private ShooterState m_setpoint = ShooterState.STARTING_STATE;

  // private final SingleJointedMechanismVisualizer m_setpointVisualizer =
  //     new SingleJointedMechanismVisualizer(
  //         "Shooter",
  //         "Setpoint",
  //         Constants.Shooter.SHOOTER_LENGTH_METERS,
  //         Constants.Shooter.PIVOT_POSE,
  //         ShooterState.STARTING_STATE.angle());
  // private final SingleJointedMechanismVisualizer m_measuredVisualizer =
  //     new SingleJointedMechanismVisualizer(
  //         "Shooter",
  //         "Measured",
  //         Constants.Shooter.SHOOTER_LENGTH_METERS,
  //         Constants.Shooter.PIVOT_POSE,
  //         ShooterState.STARTING_STATE.angle());

  private static final LoggedTunableNumber erectorKs = new LoggedTunableNumber("ErectorKs");
  private static final LoggedTunableNumber erectorKg = new LoggedTunableNumber("ErectorKg");
  private static final LoggedTunableNumber erectorKv = new LoggedTunableNumber("ErectorKv");
  private static final LoggedTunableNumber erectorKa = new LoggedTunableNumber("ErectorKa");

  private static final LoggedTunableNumber erectorKp = new LoggedTunableNumber("ErectorKp");
  private static final LoggedTunableNumber erectorKi = new LoggedTunableNumber("ErectorKi");
  private static final LoggedTunableNumber erectorKd = new LoggedTunableNumber("ErectorKd");

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

  private ArmFeedforward m_erectorFeedforward = new ArmFeedforward(0, 0, 0);
  private final PIDController m_erectorFeedback = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward m_shooterModuleFeedforward = new SimpleMotorFeedforward(0, 0);
  private final PIDController m_shooterModuleFeedback = new PIDController(0, 0, 0);

  private final SysIdRoutine m_erectorCharacterizationRoutine;
  private final SysIdRoutine m_shooterCharacterizationRoutine;

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_2024_COMP -> {
        erectorKs.initDefault(0.0); // TODO
        erectorKg.initDefault(0.0); // TODO
        erectorKv.initDefault(0.0); // TODO
        erectorKa.initDefault(0.0); // TODO
        erectorKp.initDefault(0.0); // TODO
        erectorKi.initDefault(0.0); // TODO
        erectorKd.initDefault(0.0); // TODO
        shooterModuleKs.initDefault(0.0); // TODO
        shooterModuleKv.initDefault(0.0); // TODO
        shooterModuleKp.initDefault(0.0); // TODO
        shooterModuleKi.initDefault(0.0); // TODO
        shooterModuleKd.initDefault(0.0); // TODO
      }
      case ROBOT_SIMBOT -> {
        erectorKs.initDefault(0.0); // TODO
        erectorKg.initDefault(0.0); // TODO
        erectorKv.initDefault(0.0); // TODO
        erectorKa.initDefault(0.0); // TODO
        erectorKp.initDefault(0.0); // TODO
        erectorKi.initDefault(0.0); // TODO
        erectorKd.initDefault(0.0); // TODO
        shooterModuleKs.initDefault(0.0); // TODO
        shooterModuleKv.initDefault(0.0); // TODO
        shooterModuleKp.initDefault(0.0); // TODO
        shooterModuleKi.initDefault(0.0); // TODO
        shooterModuleKd.initDefault(0.0); // TODO
      }
    }
  }

  public ShooterBase(ErectorIO erectorIO, ShooterModuleIO shooterModuleIO, KickupIO kickupIO) {
    this.m_erectorIO = erectorIO;
    this.m_shooterModuleIO = shooterModuleIO;
    this.m_kickupIO = kickupIO;

    m_erectorIO.setBrakeMode(true);

    // TODO, determine ramp rates and step rates due to the erector having much less range to
    //  characterize
    m_erectorCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/ErectorSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> m_erectorIO.setVoltage(voltage.in(Volts)), null, this, "Erector"));

    m_shooterCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/ShooterSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> m_shooterModuleIO.setVoltage(voltage.in(Volts)),
                null,
                this,
                "Shooter"));
  }

  @Override
  public void periodic() {
    m_shooterModuleIO.updateInputs(m_shooterModuleInputs);
    m_erectorIO.updateInputs(m_erectorInputs);
    m_kickupIO.updateInputs(m_kickupInputs);

    Logger.processInputs("Shooter/Erector", m_erectorInputs);
    Logger.processInputs("Shooter/Module", m_shooterModuleInputs);
    Logger.processInputs("Shooter/Kickup", m_kickupInputs);

    if (erectorKs.hasChanged(0)
        || erectorKg.hasChanged(0)
        || erectorKv.hasChanged(0)
        || erectorKa.hasChanged(0)) {
      m_erectorFeedforward =
          new ArmFeedforward(erectorKs.get(), erectorKg.get(), erectorKv.get(), erectorKa.get());
    }
    if (erectorKp.hasChanged(0) || erectorKi.hasChanged(0) || erectorKd.hasChanged(0)) {
      m_erectorFeedback.setPID(erectorKp.get(), erectorKi.get(), erectorKd.get());
    }

    if (shooterModuleKs.hasChanged(0) || shooterModuleKv.hasChanged(0)) {
      m_shooterModuleFeedforward =
          new SimpleMotorFeedforward(shooterModuleKs.get(), shooterModuleKv.get());
    }
    if (shooterModuleKp.hasChanged(0)
        || shooterModuleKi.hasChanged(0)
        || shooterModuleKd.hasChanged(0)) {
      m_shooterModuleFeedback.setPID(
          shooterModuleKp.get(), shooterModuleKi.get(), shooterModuleKd.get());
    }

    // Update setpoint and command IO layers
    if (DriverStation.isDisabled()) {
      // Reset visualizer to default state when disabled
      m_setpoint = ShooterState.TRAVEL_STATE;

      // Disable the erector and shooter
      m_shooterModuleIO.setVoltage(0.0);
      m_erectorIO.setVoltage(0.0);
      m_kickupIO.setVoltage(0.0);
    } else if (m_setpoint != null) {
      double erectorMeasurement = m_erectorInputs.absoluteAngle.getRadians();
      double erectorSetpoint = m_setpoint.angle().getRadians();

      m_erectorIO.setVoltage(
          MathUtil.clamp(
              m_erectorFeedforward.calculate(erectorSetpoint, 0.0)
                  + m_erectorFeedback.calculate(erectorMeasurement, erectorSetpoint),
              -12,
              12));

      double shooterMeasurement = getShooterVelocityMetersPerSecond();
      double shooterSetpoint = m_setpoint.shooterVelocityMetersPerSecond();

      m_shooterModuleIO.setVoltage(
          MathUtil.clamp(
              m_shooterModuleFeedforward.calculate(shooterSetpoint)
                  + m_shooterModuleFeedback.calculate(shooterMeasurement, shooterSetpoint),
              -12,
              12));

      m_kickupIO.setVoltage(m_setpoint.kickupPercent() * 12.0);
    }

    // TODO do visualizer
    // m_setpointVisualizer.update(m_setpoint.angle());
    // m_measuredVisualizer.update(m_erectorInputs.absoluteAngle);
  }

  public void setSetpoint(ShooterState state) {
    m_setpoint = state;
  }

  public ShooterState getSetpoint() {
    return m_setpoint;
  }

  public ShooterState getCurrentState() {
    return new ShooterState(
        m_erectorInputs.absoluteAngle,
        getShooterVelocityMetersPerSecond(),
        getKickupVelocityMetersPerSecond());
  }

  public boolean atSetpoint() {
    return getSetpoint().equals(getCurrentState());
  }

  @AutoLogOutput(key = "Shooter/VelocityMetersPerSec")
  public double getShooterVelocityMetersPerSecond() {
    return m_shooterModuleInputs.velocityRadPerSec * SHOOTER_RADIUS_METERS;
  }

  @AutoLogOutput(key = "Shooter/KickupVelocityMetersPerSec")
  public double getKickupVelocityMetersPerSecond() {
    return m_kickupInputs.velocityRadPerSec * KICKUP_RADIUS_METERS;
  }

  public boolean holdingNote() {
    return m_kickupInputs.forwardBeamBreakBroken && m_kickupInputs.rearBeamBreakBroken;
  }

  public Command characterizeErectorQuasistatic(SysIdRoutine.Direction direction) {
    return handleCharacterization()
        .andThen(m_erectorCharacterizationRoutine.quasistatic(direction));
  }

  public Command characterizeErectorDynamic(SysIdRoutine.Direction direction) {
    return handleCharacterization().andThen(m_erectorCharacterizationRoutine.dynamic(direction));
  }

  public Command characterizeShooterQuasistatic(SysIdRoutine.Direction direction) {
    return handleCharacterization()
        .andThen(m_shooterCharacterizationRoutine.quasistatic(direction));
  }

  public Command characterizeShooterDynamic(SysIdRoutine.Direction direction) {
    return handleCharacterization().andThen(m_shooterCharacterizationRoutine.dynamic(direction));
  }

  /**
   * Stops the subsystem from going to the setpoint
   *
   * @return Command that sets up the subsystem for characterization. This should be run before the
   *     characterization command.
   */
  private Command handleCharacterization() {
    return Commands.runOnce(
        () -> {
          m_setpoint = null;
          m_erectorIO.setVoltage(0);
          m_kickupIO.setVoltage(0);
          m_shooterModuleIO.setVoltage(0);
        },
        this);
  }
}
