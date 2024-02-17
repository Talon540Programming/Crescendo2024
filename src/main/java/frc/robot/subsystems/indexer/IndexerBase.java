package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.indexer.IndexerDynamics.IndexerState;
import frc.robot.util.LoggedTunableNumber;

public class IndexerBase extends SubsystemBase {
    public static double INDEXER_RADIUS_METERS = Units.inchesToMeters(2);

    private final IndexerIO m_indexerIO;
    private final IndexerIOInputsAutoLogged m_indexerInputs = new IndexerIOInputsAutoLogged();

    private IndexerState m_setpoint = IndexerState.STARTING_STATE;

    private static final LoggedTunableNumber indexerKp = new LoggedTunableNumber("IndexerKp");
    private static final LoggedTunableNumber indexerKi = new LoggedTunableNumber("IndexerKi");
    private static final LoggedTunableNumber indexerKd = new LoggedTunableNumber("IndexerKd");

    private final PIDController m_indexerFeedback = new PIDController(0, 0, 0);

    static {
        switch (Constants.getRobotType()) {
            case ROBOT_2024_COMP -> {
                indexerKp.initDefault(0.0); //TODO
                indexerKi.initDefault(0.0); //TODO
                indexerKd.initDefault(0.0); //TODO
            }
            case ROBOT_SIMBOT -> {
                indexerKp.initDefault(0.0); //TODO
                indexerKi.initDefault(0.0); //TODO
                indexerKd.initDefault(0.0); //TODO
            }
        }
    }

    public IndexerBase(IndexerIO indexerIO) {
        this.m_indexerIO = indexerIO;
    }

    @Override
    public void periodic() {
        m_indexerIO.updateInputs(m_indexerInputs);

        Logger.processInputs("Indexer/Indexer", m_indexerInputs);

        if (indexerKp.hasChanged(0) || indexerKi.hasChanged(0) || indexerKd.hasChanged(0)) {
            m_indexerFeedback.setPID(indexerKp.get(), indexerKi.get(), indexerKd.get());
        }

        if (DriverStation.isDisabled()) {
            m_setpoint = IndexerState.TRAVEL_STATE;

            m_indexerIO.setVoltage(0.0);
        } else {

            double indexerMeasurement = m_indexerInputs.velocityRadPerSec * INDEXER_RADIUS_METERS;
            double indexerSetpoint = m_setpoint.indexerVelocityMetersPerSecond();

            m_indexerIO.setVoltage(
                MathUtil.clamp(m_indexerFeedback.calculate(indexerMeasurement, indexerSetpoint), -12, 12));
        }
    }

    public void setIndexerState(IndexerState state) {
        m_setpoint = state;
    }
}
