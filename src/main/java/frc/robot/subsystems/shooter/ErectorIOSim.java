package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.ShooterDynamics.ShooterState;

public class ErectorIOSim implements ErectorIO {
  private final SingleJointedArmSim m_sim;
  private double appliedVoltage = 0.0;

  public ErectorIOSim() {
    m_sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            ShooterBase.ERECTOR_GEARING,
            0.87,
            Constants.Shooter.SHOOTER_LENGTH_METERS,
            Constants.Shooter.MIN_SHOOTER_ANGLE.getRadians(),
            Constants.Shooter.MAX_SHOOTER_ANGLE.getRadians(),
            true,
            ShooterState.STARTING_STATE.angle().getRadians());
  }

  @Override
  public void updateInputs(ErectorIOInputs inputs) {
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
