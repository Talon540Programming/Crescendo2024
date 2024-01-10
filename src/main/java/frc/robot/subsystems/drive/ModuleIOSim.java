package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two DCMotor sims for the drive and turn motors, with the absolute position initialized to
 * a random value. The DCMotor sims are not physically accurate, but provide a decent approximation
 * for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim m_driveSim;
  private final DCMotorSim m_turnSim;

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    this.m_driveSim = new DCMotorSim(DCMotor.getNEO(1), Constants.Drivetrain.kDriveGearing, 0.025);
    this.m_turnSim = new DCMotorSim(DCMotor.getNEO(1), Constants.Drivetrain.kTurnGearing, 0.004);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    m_driveSim.update(Constants.kLoopPeriodSecs);
    m_turnSim.update(Constants.kLoopPeriodSecs);

    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(m_driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(m_turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(m_turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(m_turnSim.getCurrentDrawAmps())};

    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_turnSim.setInputVoltage(turnAppliedVolts);
  }
}
