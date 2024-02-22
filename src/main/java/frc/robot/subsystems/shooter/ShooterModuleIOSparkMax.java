package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareIds;
import frc.robot.util.SparkMaxUtils;

public class ShooterModuleIOSparkMax implements ShooterModuleIO {
  private final CANSparkMax m_leader;
  private final CANSparkMax m_follower;

  private final RelativeEncoder m_encoder;

  public ShooterModuleIOSparkMax() {
    m_leader =
        new CANSparkMax(HardwareIds.COMP_2024.kShooterLeaderId, CANSparkMax.MotorType.kBrushless);
    m_follower =
        new CANSparkMax(HardwareIds.COMP_2024.kShooterFollowerId, CANSparkMax.MotorType.kBrushless);

    m_leader.restoreFactoryDefaults();
    m_follower.restoreFactoryDefaults();

    m_leader.setCANTimeout(250);
    m_follower.setCANTimeout(250);

    m_leader.setInverted(true);

    m_leader.setSmartCurrentLimit(40);
    m_follower.setSmartCurrentLimit(40);
    m_leader.enableVoltageCompensation(12);
    m_follower.enableVoltageCompensation(12);

    m_encoder = m_leader.getEncoder();
    m_encoder.setPosition(0.0);
    m_encoder.setMeasurementPeriod(10);
    m_encoder.setAverageDepth(2);

    m_leader.setCANTimeout(0);
    m_follower.setCANTimeout(0);

    m_leader.burnFlash();
    m_follower.burnFlash();

    SparkMaxUtils.disableSensorFrames(m_leader, m_follower);
    SparkMaxUtils.configureFollowers(m_follower);
    // Report velocity at a faster rate for PID
    // m_leader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);

    m_follower.follow(m_leader);
  }

  @Override
  public void updateInputs(ShooterModuleIOInputs inputs) {
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity())
            / ShooterBase.SHOOTER_GEARING;
    inputs.appliedVolts = m_leader.getAppliedOutput() * m_leader.getBusVoltage();
    inputs.currentAmps = new double[] {m_leader.getOutputCurrent(), m_follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
  }
}
