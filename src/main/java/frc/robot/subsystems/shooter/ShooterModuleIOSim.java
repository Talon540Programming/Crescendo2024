package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;

public class ShooterModuleIOSim implements ShooterModuleIO {
  private final FlywheelSim m_sim;
  private double appliedVoltage = 0.0;

  public ShooterModuleIOSim() {
    m_sim = new FlywheelSim(DCMotor.getNEO(1), ShooterBase.SHOOTER_GEARING, 0.01072712);
  }

  @Override
  public void updateInputs(ShooterModuleIOInputs inputs) {
    m_sim.update(Constants.kLoopPeriodSecs);

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
