package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;

public class IndexerIOSim implements IndexerIO {
  private final FlywheelSim m_sim;
  private double appliedVoltage = 0.0;

  public IndexerIOSim() {
    m_sim = new FlywheelSim(DCMotor.getNeo550(1), IntakeBase.INDEXER_GEARING, 0.01123894);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    m_sim.update(Constants.kLoopPeriodSecs);

    inputs.velocityRadPerSec = m_sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.currentAmps = new double[] {m_sim.getCurrentDrawAmps()};
    inputs.beamBreakBroken = false;
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    m_sim.setInputVoltage(appliedVoltage);
  }
}
