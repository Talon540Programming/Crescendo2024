package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;

public class ShooterModuleIOSim implements ShooterModuleIO {
  private final FlywheelSim m_topSim;
  private final FlywheelSim m_bottomSim;

  private final PIDController m_topController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController m_bottomController = new PIDController(0.0, 0.0, 0.0);

  private double topAppliedVoltage = 0.0;
  private double bottomAppliedVoltage = 0.0;

  private Double topSetpoint;
  private Double bottomSetpoint;

  private double topFeedforward;
  private double bottomFeedforward;

  public ShooterModuleIOSim() {
    m_topSim = new FlywheelSim(DCMotor.getNEO(1), ShooterBase.SHOOTER_GEARING, 0.01072712);
    m_bottomSim = new FlywheelSim(DCMotor.getNEO(1), ShooterBase.SHOOTER_GEARING, 0.01072712);
  }

  @Override
  public void updateInputs(ShooterModuleIOInputs inputs) {
    m_topSim.update(Constants.kLoopPeriodSecs);
    m_bottomSim.update(Constants.kLoopPeriodSecs);

    if (topSetpoint != null && bottomSetpoint != null) {
      setVoltage(
          m_topController.calculate(m_topSim.getAngularVelocityRadPerSec(), topSetpoint)
              + topFeedforward,
          m_bottomController.calculate(m_bottomSim.getAngularVelocityRadPerSec(), bottomSetpoint)
              + bottomFeedforward);
    }

    inputs.topPositionRad +=
        Units.radiansToRotations(
            m_topSim.getAngularVelocityRadPerSec() * Constants.kLoopPeriodSecs);
    inputs.topVelocityRadPerSecond = m_topSim.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVoltage;
    inputs.topCurrentAmps = m_topSim.getCurrentDrawAmps();

    inputs.bottomPositionRad +=
        Units.radiansToRotations(
            m_bottomSim.getAngularVelocityRadPerSec() * Constants.kLoopPeriodSecs);
    inputs.bottomVelocityRadPerSecond = m_bottomSim.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = bottomAppliedVoltage;
    inputs.bottomCurrentAmps = m_bottomSim.getCurrentDrawAmps();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_topController.setPID(kP, kI, kD);
    m_bottomController.setPID(kP, kI, kD);
  }

  @Override
  public void runSetpoint(
      double topSetpointRadPerSec,
      double bottomSetpointRadPerSec,
      double topFeedforwardVoltage,
      double bottomFeedforwardVoltage) {
    topSetpoint = topSetpointRadPerSec;
    bottomSetpoint = bottomSetpointRadPerSec;
    topFeedforward = topFeedforwardVoltage;
    bottomFeedforward = bottomFeedforwardVoltage;
  }

  @Override
  public void runCharacterizationVoltage(double voltage) {
    topSetpoint = null;
    bottomSetpoint = null;
    setVoltage(voltage, voltage);
  }

  @Override
  public void stop() {
    setVoltage(0.0, 0.0);
  }

  private void setVoltage(double topVolts, double bottomVolts) {
    topAppliedVoltage = MathUtil.clamp(topVolts, -12, 12);
    bottomAppliedVoltage = MathUtil.clamp(bottomVolts, -12, 12);

    m_topSim.setInputVoltage(topAppliedVoltage);
    m_bottomSim.setInputVoltage(bottomAppliedVoltage);
  }
}
