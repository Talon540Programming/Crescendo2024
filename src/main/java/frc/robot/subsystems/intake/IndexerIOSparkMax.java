package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareIds;

public class IndexerIOSparkMax implements IndexerIO {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;

  public IndexerIOSparkMax() {
    m_motor =
        new CANSparkMax(HardwareIds.COMP_2024.kIndexId, CANSparkLowLevel.MotorType.kBrushless);

    m_motor.restoreFactoryDefaults();

    m_motor.setCANTimeout(250);

    m_motor.setInverted(true);

    m_motor.setSmartCurrentLimit(40);
    m_motor.enableVoltageCompensation(12);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0.0);
    m_encoder.setMeasurementPeriod(10);
    m_encoder.setAverageDepth(2);

    m_motor.setCANTimeout(0);

    m_motor.burnFlash();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());
    inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.currentAmps = new double[] {m_motor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
