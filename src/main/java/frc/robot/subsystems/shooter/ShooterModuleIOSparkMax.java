package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareIds;
import frc.robot.util.SparkMaxUtils;

public class ShooterModuleIOSparkMax implements ShooterModuleIO {
  private final CANSparkMax m_top;
  private final CANSparkMax m_bottom;

  private final RelativeEncoder m_topEncoder;
  private final RelativeEncoder m_bottomEncoder;

  private final SparkPIDController m_topController;
  private final SparkPIDController m_bottomController;

  public ShooterModuleIOSparkMax() {
    m_top =
        new CANSparkMax(HardwareIds.COMP_2024.kShooterLeaderId, CANSparkMax.MotorType.kBrushless);
    m_bottom =
        new CANSparkMax(HardwareIds.COMP_2024.kShooterFollowerId, CANSparkMax.MotorType.kBrushless);

    m_top.restoreFactoryDefaults();
    m_bottom.restoreFactoryDefaults();

    m_top.setCANTimeout(250);
    m_bottom.setCANTimeout(250);

    m_top.setInverted(true);
    m_bottom.setInverted(true);

    m_top.setSmartCurrentLimit(40);
    m_bottom.setSmartCurrentLimit(40);
    m_top.enableVoltageCompensation(12);
    m_bottom.enableVoltageCompensation(12);

    m_topEncoder = m_top.getEncoder();
    m_topEncoder.setPosition(0.0);
    m_topEncoder.setMeasurementPeriod(10);
    m_topEncoder.setAverageDepth(2);

    m_bottomEncoder = m_top.getEncoder();
    m_bottomEncoder.setPosition(0.0);
    m_bottomEncoder.setMeasurementPeriod(10);
    m_bottomEncoder.setAverageDepth(2);

    m_topController = m_top.getPIDController();
    m_bottomController = m_bottom.getPIDController();

    m_top.setIdleMode(CANSparkBase.IdleMode.kCoast);
    m_bottom.setIdleMode(CANSparkBase.IdleMode.kCoast);

    m_top.setCANTimeout(0);
    m_bottom.setCANTimeout(0);

    m_top.burnFlash();
    m_bottom.burnFlash();

    SparkMaxUtils.disableSensorFrames(m_top, m_bottom);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_topController.setP(kP);
    m_topController.setI(kI);
    m_topController.setD(kD);
    m_bottomController.setP(kP);
    m_bottomController.setI(kI);
    m_bottomController.setD(kD);
  }

  @Override
  public void runSetpoint(
      double topSetpointRadPerSec,
      double bottomSetpointRadPerSec,
      double topFeedforwardVoltage,
      double bottomFeedforwardVoltage) {
    m_topController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(topSetpointRadPerSec)
            * ShooterBase.SHOOTER_GEARING,
        CANSparkBase.ControlType.kVelocity,
        0,
        topFeedforwardVoltage,
        SparkPIDController.ArbFFUnits.kVoltage);
    m_bottomController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(bottomSetpointRadPerSec)
            * ShooterBase.SHOOTER_GEARING,
        CANSparkBase.ControlType.kVelocity,
        0,
        bottomFeedforwardVoltage,
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void runCharacterizationVoltage(double voltage) {
    m_top.setVoltage(MathUtil.clamp(voltage, -12, 12));
    m_bottom.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void updateInputs(ShooterModuleIOInputs inputs) {
    inputs.topPositionRad =
        Units.rotationsToRadians(m_topEncoder.getPosition()) / ShooterBase.SHOOTER_GEARING;
    inputs.topVelocityRadPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(m_topEncoder.getVelocity())
            / ShooterBase.SHOOTER_GEARING;
    inputs.topAppliedVolts = m_top.getAppliedOutput() * m_top.getBusVoltage();
    inputs.topCurrentAmps = m_top.getOutputCurrent();

    inputs.bottomPositionRad =
        Units.rotationsToRadians(m_bottomEncoder.getPosition()) / ShooterBase.SHOOTER_GEARING;
    inputs.bottomVelocityRadPerSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(m_bottomEncoder.getVelocity())
            / ShooterBase.SHOOTER_GEARING;
    inputs.bottomAppliedVolts = m_bottom.getAppliedOutput() * m_bottom.getBusVoltage();
    inputs.bottomCurrentAmps = m_bottom.getOutputCurrent();
  }
}
