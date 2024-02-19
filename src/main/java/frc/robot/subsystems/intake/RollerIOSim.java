package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;

public class RollerIOSim implements RollerIO {
  private final FlywheelSim m_sim;
  private double appliedVoltage = 0.0;

  public RollerIOSim() {
    m_sim = new FlywheelSim(DCMotor.getNeo550(1), IntakeBase.ROLLER_GEARING, 0.14159265);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    m_sim.update(Constants.kLoopPeriodSecs);

    inputs.BeamBreakBroken = false;
    inputs.velocityRadPerSec = m_sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.currentAmps = new double[] {m_sim.getCurrentDrawAmps()};
  }

  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    m_sim.setInputVoltage(appliedVoltage);
  }
}
