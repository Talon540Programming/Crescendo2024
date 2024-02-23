package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;

public class KickupIOSim implements KickupIO {
  private final FlywheelSim m_sim;
  private double appliedVoltage = 0.0;

  public KickupIOSim() {
    m_sim = new FlywheelSim(DCMotor.getNeo550(1), ShooterBase.KICKUP_GEARING, 0.00970256);
  }

  @Override
  public void updateInputs(KickupIOInputs inputs) {
    m_sim.update(Constants.kLoopPeriodSecs);

    inputs.forwardBeamBreakBroken = false;
    inputs.rearBeamBreakBroken = false;
    inputs.velocityRadPerSec = m_sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.currentAmps = new double[] {m_sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    m_sim.setInputVoltage(appliedVoltage);
  }
}
