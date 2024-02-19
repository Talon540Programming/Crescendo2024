package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareIds;

public class RollerIOSparkMax implements RollerIO {
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;

  private final DigitalInput m_beamBreak;

  public RollerIOSparkMax() {
    m_motor =
        new CANSparkMax(
            HardwareIds.COMP_2024.kIntakeRollersId, CANSparkLowLevel.MotorType.kBrushless);

    m_beamBreak = new DigitalInput(HardwareIds.COMP_2024.kIntakeBeamBreakPort);

    m_motor.restoreFactoryDefaults();
    m_motor.setCANTimeout(250);
    m_motor.setInverted(false);

    m_motor.setSmartCurrentLimit(40); // TODO
    m_motor.enableVoltageCompensation(12); // TODO

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0.0);
    m_encoder.setMeasurementPeriod(10);
    m_encoder.setAverageDepth(2);

    m_motor.setCANTimeout(0);
    m_motor.burnFlash();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.BeamBreakBroken = !m_beamBreak.get();
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity())
            / IntakeBase.ROLLER_GEARING;
    inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.currentAmps = new double[] {m_motor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
