package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;

public class WristIOSim implements WristIO {
  private final SingleJointedArmSim m_sim;
  private double appliedVoltage = 0.0;

  public WristIOSim() {
    m_sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            IntakeBase.WRIST_GEARING,
            0.87,
            Constants.Intake.kIntakeLengthMeters,
            Constants.Intake.kMinIntakeAngle.getRadians(),
            Constants.Intake.kMaxIntakeAngle.getRadians(),
            true,
            Constants.Intake.kMaxIntakeAngle.getRadians());
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    m_sim.update(Constants.kLoopPeriodSecs);

    inputs.absoluteAngle = Rotation2d.fromRadians(m_sim.getAngleRads());
    inputs.velocityRadPerSec = m_sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.currentAmps = new double[] {m_sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    m_sim.setInputVoltage(appliedVoltage);
  }
}
