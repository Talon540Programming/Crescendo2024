package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.IntakeDynamics.IntakeState;
import frc.robot.util.LoggedTunableNumber;

public class IntakeBase extends SubsystemBase {
    public static final double WRIST_GEARING = (36.0 / 1.0);
    public static final double ROLLER_GEARING = (5.0 / 1.0);
    public static final double WRIST_RADIUS_METERS = Units.inchesToMeters(0); //TODO
    public static final double ROLLER_RADIUS_METERS = Units.inchesToMeters(0); //TODO

    private final WristIO m_wristIO;
    private final WristIOInputsAutoLogged m_wristInputs = new WristIOInputsAutoLogged();

    private final RollerIO m_rollerIO;
    private final RollerIOInputsAutoLogged m_rollerInputs = new RollerIOInputsAutoLogged();

    private IntakeState m_setpoint = IntakeState.STARTING_STATE;


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

    public IntakeBase(WristIO wristIO, RollerIO rollerIO) {
        this.m_wristIO = wristIO;
        this.m_rollerIO = rollerIO;

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

        Logger.processInputs("Intake/Wrist", m_wristInputs);
        Logger.processInputs("Intake/Roller", m_wristInputs);

        if (wristKs.hasChanged(0) || wristKg.hasChanged(0) || wristKv.hasChanged(0) || wristKa.hasChanged(0)) {
            m_wristFeedforward = new ArmFeedforward(wristKs.get(), wristKg.get(), wristKv.get(), wristKa.get());
        }
        if (wristKp.hasChanged(0) || wristKi.hasChanged(0) || wristKd.hasChanged(0)) {
            m_wristFeedback.setPID(wristKp.get(), wristKi.get(), wristKd.get());
        }


        if (DriverStation.isDisabled()) {
            m_setpoint = IntakeState.TRAVEL_STATE;

            m_wristIO.setVoltage(0);
            m_rollerIO.setVoltage(0);
        } else if (m_setpoint != null) {


            double wristMeasurement = m_wristInputs.absoluteAngle.getRadians();
            double wristSetpoint = m_setpoint.angle().getRadians();

            m_wristIO.setVoltage(
                MathUtil.clamp(
                    m_wristFeedforward.calculate(wristSetpoint, 0.0)
                         + m_wristFeedback.calculate(wristMeasurement, wristSetpoint),
                    -12,
                    12));
            
            
            m_rollerIO.setVoltage(m_setpoint.rollerPercent()*12.0);
        }
    }

    public void setSetpoint(IntakeState state) {
        m_setpoint = state;
    }

    public IntakeState getSetpoint() {
        return m_setpoint;
    }

    public IntakeState getCurrentState() {
        return new IntakeState(
            m_wristInputs.absoluteAngle,
            getRollerVelocityMetersPerSec());
    }

    public boolean atSetpoint() {
        return getSetpoint().equals(getCurrentState());
    }

    @AutoLogOutput(key="Intake/RollerVelocityMetersPerSec")
    public double getRollerVelocityMetersPerSec() {
        return m_rollerInputs.velocityRadPerSec * ROLLER_RADIUS_METERS;
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
                m_setpoint = null;
                m_wristIO.setVoltage(0);
                m_rollerIO.setVoltage(0);
            },
            this);
    }
}
