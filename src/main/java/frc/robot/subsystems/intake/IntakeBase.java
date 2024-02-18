package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeBase extends SubsystemBase {
    public static final double WRIST_GEARING = (36.0 / 1.0);
    public static final double ROLLER_GEARING = (5.0 / 1.0);
    public static final double WRIST_RADIUS_METERS = Units.inchesToMeters(0); //TODO
    public static final double ROLLER_RADIUS_METERS = Units.inchesToMeters(0); //TODO
    public static final double INDEXER_RADIUS_METERS = Units.inchesToMeters(2);

    private final IndexerIO m_indexerIO;
    private final IndexerIOInputsAutoLogged m_indexerInputs = new IndexerIOInputsAutoLogged();

    private final WristIO m_wristIO;
    private final WristIOInputsAutoLogged m_wristInputs = new WristIOInputsAutoLogged();

    private final RollerIO m_rollerIO;
    private final RollerIOInputsAutoLogged m_rollerInputs = new RollerIOInputsAutoLogged();

    private Rotation2d m_wristSetpoint = Constants.Intake.kMaxIntakeAngle;


    private static final LoggedTunableNumber wristKs = new LoggedTunableNumber("WristKs");
    private static final LoggedTunableNumber wristKg = new LoggedTunableNumber("WristKg");
    private static final LoggedTunableNumber wristKv = new LoggedTunableNumber("WristKv");
    private static final LoggedTunableNumber wristKa = new LoggedTunableNumber("WristKa");

    private static final LoggedTunableNumber wristKp = new LoggedTunableNumber("WristKp");
    private static final LoggedTunableNumber wristKi = new LoggedTunableNumber("WristKi");
    private static final LoggedTunableNumber wristKd = new LoggedTunableNumber("WristKd");

    private ArmFeedforward m_wristFeedforward = new ArmFeedforward(0, 0, 0);
    private final PIDController m_wristFeedback = new PIDController(0, 0, 0);
    
    private final SysIdRoutine m_wristCharacterizationRoutine;

    static {
        switch (Constants.getRobotType()) {
            case ROBOT_2024_COMP -> {
                wristKs.initDefault(0.0); //TODO
                wristKg.initDefault(0.0); //TODO
                wristKv.initDefault(0.0); //TODO
                wristKa.initDefault(0.0); //TODO
                wristKp.initDefault(0.0); //TODO
                wristKi.initDefault(0.0); //TODO
                wristKd.initDefault(0.0); //TODO
            }
            case ROBOT_SIMBOT -> {
                wristKs.initDefault(0.0); //TODO
                wristKg.initDefault(0.0); //TODO
                wristKv.initDefault(0.0); //TODO
                wristKa.initDefault(0.0); //TODO
                wristKp.initDefault(0.0); //TODO
                wristKi.initDefault(0.0); //TODO
                wristKd.initDefault(0.0); //TODO
            }
        }
    }

    public IntakeBase(WristIO wristIO, RollerIO rollerIO, IndexerIO indexerIO) {
        this.m_wristIO = wristIO;
        this.m_rollerIO = rollerIO;
        this.m_indexerIO = indexerIO;

        m_wristIO.setBrakeMode(true);

        //TODO wrist FF characterization
        m_wristCharacterizationRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Intake/WristSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> m_wristIO.setVoltage(voltage.in(Volts)),
                    null,
                    this,
                    "Wrist"));
    }

    @Override
    public void periodic() {
        m_wristIO.updateInputs(m_wristInputs);
        m_rollerIO.updateInputs(m_rollerInputs);
        m_indexerIO.updateInputs(m_indexerInputs);

        Logger.processInputs("Intake/Wrist", m_wristInputs);
        Logger.processInputs("Intake/Roller", m_rollerInputs);
        Logger.processInputs("Intake/Indexer", m_indexerInputs);

        if (wristKs.hasChanged(0) || wristKg.hasChanged(0) || wristKv.hasChanged(0) || wristKa.hasChanged(0)) {
            m_wristFeedforward = new ArmFeedforward(wristKs.get(), wristKg.get(), wristKv.get(), wristKa.get());
        }
        if (wristKp.hasChanged(0) || wristKi.hasChanged(0) || wristKd.hasChanged(0)) {
            m_wristFeedback.setPID(wristKp.get(), wristKi.get(), wristKd.get());
        }


        if (DriverStation.isDisabled()) {
            m_wristSetpoint = Constants.Intake.kMinIntakeAngle;

            m_wristIO.setVoltage(0);
            m_rollerIO.setVoltage(0);
            m_indexerIO.setVoltage(0);
        } else if (m_wristSetpoint != null) {


            double wristMeasurement = m_wristInputs.absoluteAngle.getRadians();
            double wristSetpoint = m_wristSetpoint.getRadians();

            m_wristIO.setVoltage(
                MathUtil.clamp(
                    m_wristFeedforward.calculate(wristSetpoint, 0.0)
                         + m_wristFeedback.calculate(wristMeasurement, wristSetpoint),
                    -12,
                    12));
        }
    }

    public void setWristSetpoint(Rotation2d state) {
        m_wristSetpoint = state;
    }

    public void setRollerVoltage(double volts) {
        m_rollerIO.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    public void setIndexerVoltage(double volts) {
        m_indexerIO.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    public Rotation2d getWristSetpoint() {
        return m_wristSetpoint;
    }

    public Rotation2d getCurrentWristState() {
        return m_wristInputs.absoluteAngle;
    }

    public boolean atWristSetpoint() {
        return getWristSetpoint().equals(getCurrentWristState());
    }

    @AutoLogOutput(key="Intake/RollerVelocityMetersPerSec")
    public double getRollerVelocityMetersPerSec() {
        return m_rollerInputs.velocityRadPerSec * ROLLER_RADIUS_METERS;
    }

    @AutoLogOutput(key="Intake/IndexerVelocityMetersPerSec")
    public double getIndexerVelocityMetersPerSec() {
        return m_indexerInputs.velocityRadPerSec * INDEXER_RADIUS_METERS;
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
                m_wristSetpoint = null;
                m_wristIO.setVoltage(0);
                m_rollerIO.setVoltage(0);
            },
            this);
    }
}
