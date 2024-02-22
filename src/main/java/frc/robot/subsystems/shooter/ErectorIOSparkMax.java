package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.HardwareIds;
import frc.robot.util.REVThroughBoreEncoder;
import frc.robot.util.SparkMaxUtils;

public class ErectorIOSparkMax implements ErectorIO {
  private final CANSparkMax m_leader;
  private final CANSparkMax m_follower;
  private final REVThroughBoreEncoder m_encoder;

  public ErectorIOSparkMax() {
    m_leader =
        new CANSparkMax(HardwareIds.COMP_2024.kErectorLeaderId, CANSparkMax.MotorType.kBrushless);
    m_follower =
        new CANSparkMax(HardwareIds.COMP_2024.kErectorFollowerId, CANSparkMax.MotorType.kBrushless);
    m_encoder =
        new REVThroughBoreEncoder(
            HardwareIds.COMP_2024.kErectorAbsoluteEncoderPort,
            HardwareIds.COMP_2024.kErectorRelativeEncoderAPort,
            HardwareIds.COMP_2024.kErectorRelativeEncoderBPort,
            Rotation2d.fromRadians(-0.403688129475));

    m_leader.restoreFactoryDefaults();
    m_follower.restoreFactoryDefaults();

    m_leader.setCANTimeout(250);
    m_follower.setCANTimeout(250);

    m_leader.setInverted(true);

    m_leader.setSmartCurrentLimit(40);
    m_follower.setSmartCurrentLimit(40);
    m_leader.enableVoltageCompensation(12);
    m_follower.enableVoltageCompensation(12);

    m_leader.setCANTimeout(0);
    m_follower.setCANTimeout(0);

    m_leader.burnFlash();
    m_follower.burnFlash();

    SparkMaxUtils.disableSensorFrames(m_leader, m_follower);
    SparkMaxUtils.configureFollowers(m_follower);

    m_follower.follow(m_leader);
  }

  @Override
  public void updateInputs(ErectorIOInputs inputs) {
    inputs.velocityRadPerSec = m_encoder.getVelocityRadPerSecond();
    inputs.absoluteAngle = m_encoder.getAbsolutePosition();
    inputs.appliedVolts = m_leader.getAppliedOutput() * m_leader.getBusVoltage();
    inputs.currentAmps = new double[] {m_leader.getOutputCurrent(), m_follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    m_leader.setIdleMode(enable ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    m_follower.setIdleMode(enable ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }
}
