package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterDynamics.ShooterState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SingleJointedMechanismVisualizer;
import org.littletonrobotics.junction.Logger;

public class ShooterBase extends SubsystemBase {
  public static double ERECTOR_GEARING =
      (52.0 / 12.0) * (52.0 / 18.0) * (52.0 / 18.0) * (58.0 / 24.0);
  public static double SHOOTER_GEARING = (26.0 / 52.0);
  public static double SHOOTER_RADIUS_METERS = Units.inchesToMeters(1.5);
  public static double KICKUP_GEARING = (5.0 / 1.0);
  public static double KICKUP_RADIUS_METERS = Units.inchesToMeters(1.0);

  private final ErectorIO m_erectorIO;
  private final ErectorIOInputsAutoLogged m_erectorInputs = new ErectorIOInputsAutoLogged();

  private final ShooterModuleIO m_shooterModuleIO;
  private final ShooterModuleIOInputsAutoLogged m_shooterModuleInputs =
      new ShooterModuleIOInputsAutoLogged();

  private final KickupIO m_kickupIO;
  private final KickupIOInputsAutoLogged m_kickupInputs = new KickupIOInputsAutoLogged();

  private ShooterState m_setpoint = ShooterState.STARTING_STATE;

  private final SingleJointedMechanismVisualizer m_setpointVisualizer =
      new SingleJointedMechanismVisualizer(
          "Shooter",
          "Setpoint",
          Constants.Shooter.SHOOTER_LENGTH_METERS,
          Constants.Shooter.PIVOT_POSE,
          ShooterState.STARTING_STATE.angle());
  private final SingleJointedMechanismVisualizer m_measuredVisualizer =
      new SingleJointedMechanismVisualizer(
          "Shooter",
          "Measured",
          Constants.Shooter.SHOOTER_LENGTH_METERS,
          Constants.Shooter.PIVOT_POSE,
          ShooterState.STARTING_STATE.angle());

  private static final LoggedTunableNumber erectorKs = new LoggedTunableNumber("ErectorKs");
  private static final LoggedTunableNumber erectorKg = new LoggedTunableNumber("ErectorKg");
  private static final LoggedTunableNumber erectorKv = new LoggedTunableNumber("ErectorKv");
  private static final LoggedTunableNumber erectorKa = new LoggedTunableNumber("ErectorKa");

  private static final LoggedTunableNumber erectorKp = new LoggedTunableNumber("ErectorKp");
  private static final LoggedTunableNumber erectorKi = new LoggedTunableNumber("ErectorKi");
  private static final LoggedTunableNumber erectorKd = new LoggedTunableNumber("ErectorKd");

  private static final LoggedTunableNumber shooterModuleKp =
      new LoggedTunableNumber("ShooterModuleKp");
  private static final LoggedTunableNumber shooterModuleKi =
      new LoggedTunableNumber("ShooterModuleKi");
  private static final LoggedTunableNumber shooterModuleKd =
      new LoggedTunableNumber("ShooterModuleKd");

  private static final LoggedTunableNumber kickupKp = new LoggedTunableNumber("KickupKp");
  private static final LoggedTunableNumber kickupKi = new LoggedTunableNumber("KickupKi");
  private static final LoggedTunableNumber kickupKd = new LoggedTunableNumber("KickupKd");

  private ArmFeedforward m_erectorFeedforward = new ArmFeedforward(0, 0, 0);
  private final PIDController m_erectorFeedback = new PIDController(0, 0, 0);
  private final PIDController m_shooterModuleFeedback = new PIDController(0, 0, 0);
  private final PIDController m_kickupFeedback = new PIDController(0, 0, 0);

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
        shooterModuleKp.initDefault(0.0); // TODO
        shooterModuleKi.initDefault(0.0); // TODO
        shooterModuleKd.initDefault(0.0); // TODO
        kickupKp.initDefault(0.0); // TODO
        kickupKi.initDefault(0.0); // TODO
        kickupKd.initDefault(0.0); // TODO
      }
      case ROBOT_SIMBOT -> {
        erectorKs.initDefault(0.0); // TODO
        erectorKg.initDefault(0.0); // TODO
        erectorKv.initDefault(0.0); // TODO
        erectorKa.initDefault(0.0); // TODO
        erectorKp.initDefault(0.0); // TODO
        erectorKi.initDefault(0.0); // TODO
        erectorKd.initDefault(0.0); // TODO
        shooterModuleKp.initDefault(0.0); // TODO
        shooterModuleKi.initDefault(0.0); // TODO
        shooterModuleKd.initDefault(0.0); // TODO
        kickupKp.initDefault(0.0); // TODO
        kickupKi.initDefault(0.0); // TODO
        kickupKd.initDefault(0.0); // TODO
      }
    }
  }

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

    if (shooterModuleKp.hasChanged(0)
        || shooterModuleKi.hasChanged(0)
        || shooterModuleKd.hasChanged(0)) {
      m_shooterModuleFeedback.setPID(
          shooterModuleKp.get(), shooterModuleKi.get(), shooterModuleKd.get());
    }

    if (kickupKp.hasChanged(0) || kickupKi.hasChanged(0) || kickupKd.hasChanged(0)) {
      m_shooterModuleFeedback.setPID(kickupKp.get(), kickupKi.get(), kickupKd.get());
    }

    // Update setpoint and command IO layers
    if (DriverStation.isDisabled()) {
      // Reset visualizer to default state when disabled
      m_setpoint = ShooterState.TRAVEL_STATE;

      // Disable the erector and shooter
      m_shooterModuleIO.setVoltage(0.0);
      m_erectorIO.setVoltage(0.0);
      m_kickupIO.setVoltage(0.0);
    } else {
      double erectorMeasurement = m_erectorInputs.absoluteAngle.getRadians();
      double erectorSetpoint = m_setpoint.angle().getRadians();

      m_erectorIO.setVoltage(
          MathUtil.clamp(
              m_erectorFeedforward.calculate(erectorSetpoint, 0.0)
                  + m_erectorFeedback.calculate(erectorMeasurement, erectorSetpoint),
              -12,
              12));

      double shooterMeasurement = m_shooterModuleInputs.velocityRadPerSec * SHOOTER_RADIUS_METERS;
      double shooterSetpoint = m_setpoint.shooterVelocityMetersPerSecond();

      m_shooterModuleIO.setVoltage(
          MathUtil.clamp(
              m_shooterModuleFeedback.calculate(shooterMeasurement, shooterSetpoint), -12, 12));

      double kickupMeasurement = m_kickupInputs.velocityRadPerSec * KICKUP_RADIUS_METERS;
      double kickupSetpoint = m_setpoint.kickupVelocityMetersPerSecond();

      m_kickupIO.setVoltage(
          MathUtil.clamp(m_kickupFeedback.calculate(kickupMeasurement, kickupSetpoint), -12, 12));
    }

    // TODO do visualizer
  }

  public void setShooterState(ShooterState state) {
    m_setpoint = state;
  }
}
