package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareIds;

public class IndexerIOSparkMax implements IndexerIO {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;

  private final DigitalInput m_beamBreak;

  public IndexerIOSparkMax() {
    m_motor =
        new SparkMax(HardwareIds.COMP_2024.kIndexerId, MotorType.kBrushless);
    m_beamBreak = new DigitalInput(HardwareIds.COMP_2024.kIntakeBeamBreakPort);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0.0);

  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity())
            / IntakeBase.INDEXER_GEARING;
    inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.currentAmps = new double[] {m_motor.getOutputCurrent()};
    inputs.beamBreakBroken = !m_beamBreak.get();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}