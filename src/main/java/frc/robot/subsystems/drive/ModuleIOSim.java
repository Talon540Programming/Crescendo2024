package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

  private final PIDController m_driveController = new PIDController(0, 0, 0);
  private final PIDController m_turnController = new PIDController(0, 0, 0);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

  private Double velocitySetpoint = null;
  private double driveFeedForwardVolts;
  private Rotation2d angleSetpoint = null;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    this.m_driveSim = new DCMotorSim(DCMotor.getNEO(1), DriveBase.kDriveGearing, 0.025);
    this.m_turnSim = new DCMotorSim(DCMotor.getNEO(1), DriveBase.kTurnGearing, 0.004);
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);

    // Equivalent of resetting the relative encoder to the absolute position
    m_turnSim.setState(turnAbsoluteInitPosition.getRadians(), 0);
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
        new Rotation2d(MathUtil.angleModulus(m_turnSim.getAngularPositionRad()));
    inputs.turnPositionRad =
        Rotation2d.fromRadians(MathUtil.angleModulus(m_turnSim.getAngularPositionRad()));
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(m_turnSim.getCurrentDrawAmps())};

    inputs.odometryPositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(
              // Convert from radians to meters
              inputs.drivePositionRad * DriveBase.kWheelRadiusMeters, inputs.turnPositionRad)
        };

    // PID setpoints need to be constantly calculated to mirror how the SparkMax handles the onboard
    // PID controller
    if (velocitySetpoint != null) {
      runDriveVolts(
          // PID coefficients are in terms of the motor's output, not the module as a whole, scale
          // the
          // inputs to account for this
          m_driveController.calculate(m_driveSim.getAngularVelocityRadPerSec(), velocitySetpoint)
              + driveFeedForwardVolts);
    }
    if (angleSetpoint != null) {
      runTurnVolts(
          // PID coefficients are in terms of the motor's output, not the module as a whole, scale
          // the
          // inputs to account for this
          m_turnController.calculate(
              MathUtil.angleModulus(m_turnSim.getAngularPositionRad()),
              angleSetpoint.getRadians()));
    }
  }

  @Override
  public void runDriveVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void runTurnVolts(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    m_driveController.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    m_turnController.setPID(kP, kI, kD);
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadPerSec, double driveFeedForwardVolts) {
    velocitySetpoint = velocityRadPerSec;
    this.driveFeedForwardVolts = driveFeedForwardVolts;
  }

  @Override
  public void runTurnAngleSetpoint(Rotation2d angle) {
    angleSetpoint = angle;
  }
}
