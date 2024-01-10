package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.util.PoseEstimator;
import java.util.Queue;

public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final CANcoder m_absoluteEncoder;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;
  private final StatusSignal<Double> m_turnAbsoluteEncoder;

  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIOSparkMax(int driveID, int turnID, int encoderID) {
    this.m_driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    this.m_turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
    this.m_absoluteEncoder = new CANcoder(encoderID);

    this.m_driveMotor.restoreFactoryDefaults();
    this.m_turnMotor.restoreFactoryDefaults();

    this.m_driveMotor.setCANTimeout(250);
    this.m_turnMotor.setCANTimeout(250);

    this.m_driveEncoder = this.m_driveMotor.getEncoder();
    this.m_turnRelativeEncoder = this.m_turnMotor.getEncoder();
    this.m_turnAbsoluteEncoder = this.m_absoluteEncoder.getAbsolutePosition();

    this.m_turnMotor.setInverted(Constants.Drivetrain.kTurnMotorInverted);
    this.m_driveMotor.setSmartCurrentLimit(40);
    this.m_turnMotor.setSmartCurrentLimit(30);
    this.m_driveMotor.enableVoltageCompensation(12.0);
    this.m_turnMotor.enableVoltageCompensation(12.0);

    this.m_driveEncoder.setPosition(0.0);
    this.m_driveEncoder.setMeasurementPeriod(10);
    this.m_driveEncoder.setAverageDepth(2);

    this.m_turnRelativeEncoder.setPosition(0.0);
    this.m_turnRelativeEncoder.setMeasurementPeriod(10);
    this.m_turnRelativeEncoder.setAverageDepth(2);

    this.m_driveMotor.setCANTimeout(0);
    this.m_turnMotor.setCANTimeout(0);

    this.m_driveMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / PoseEstimator.ODOMETRY_FREQUENCY));
    this.m_turnMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / PoseEstimator.ODOMETRY_FREQUENCY));
    this.m_turnAbsoluteEncoder.setUpdateFrequency(50);

    this.drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(m_driveEncoder::getPosition);
    this.turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(m_turnRelativeEncoder::getPosition);

    this.m_driveMotor.burnFlash();
    this.m_turnMotor.burnFlash();

    this.m_absoluteEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(m_driveEncoder.getPosition()) / Constants.Drivetrain.kDriveGearing;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveEncoder.getVelocity())
            / Constants.Drivetrain.kDriveGearing;
    inputs.driveAppliedVolts = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {m_driveMotor.getOutputCurrent()};

    // Refresh the Encoder data because it is cached. This is non-blocking.
    m_turnAbsoluteEncoder.refresh();
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(m_turnAbsoluteEncoder.getValueAsDouble());

    inputs.turnPosition =
        Rotation2d.fromRotations(
            m_turnRelativeEncoder.getPosition() / Constants.Drivetrain.kTurnGearing);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_turnRelativeEncoder.getVelocity())
            / Constants.Drivetrain.kTurnGearing;
    inputs.turnAppliedVolts = m_turnMotor.getAppliedOutput() * m_turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {m_turnMotor.getOutputCurrent()};

    inputs.odometryDrivePositionsRad =
        this.drivePositionQueue.stream()
            .mapToDouble(
                (Double value) ->
                    Units.rotationsToRadians(value) / Constants.Drivetrain.kDriveGearing)
            .toArray();
    inputs.odometryTurnPositions =
        this.turnPositionQueue.stream()
            .map(
                (Double value) ->
                    Rotation2d.fromRotations(value / Constants.Drivetrain.kTurnGearing))
            .toArray(Rotation2d[]::new);
    this.drivePositionQueue.clear();
    this.turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveMotor.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnMotor.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    m_driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    m_turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
