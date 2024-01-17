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
import frc.robot.constants.HardwareIds;
import frc.robot.util.OdometryQueueThread;
import frc.robot.util.PoseEstimator;
import frc.robot.util.TimestampedSensorMeasurement;
import java.util.Queue;

public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final CANcoder m_absoluteEncoder;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;
  private final StatusSignal<Double> m_turnAbsoluteEncoder;

  private final Queue<TimestampedSensorMeasurement<Double>> drivePositionQueue;
  private final Queue<TimestampedSensorMeasurement<Double>> turnPositionQueue;

  public ModuleIOSparkMax(int moduleIndex) {
    switch (Constants.getRobotType()) {
      case ROBOT_2023_OFFSEASON -> {
        switch (moduleIndex) {
          case 0 -> {
            this.m_driveMotor =
                new CANSparkMax(HardwareIds.OFFSEASON_2023.kFrontLeftDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new CANSparkMax(HardwareIds.OFFSEASON_2023.kFrontLeftTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.OFFSEASON_2023.kFrontLeftEncoderId);
          }
          case 1 -> {
            this.m_driveMotor =
                new CANSparkMax(
                    HardwareIds.OFFSEASON_2023.kFrontRightDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new CANSparkMax(HardwareIds.OFFSEASON_2023.kFrontRightTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.OFFSEASON_2023.kFrontRightEncoderId);
          }
          case 2 -> {
            this.m_driveMotor =
                new CANSparkMax(HardwareIds.OFFSEASON_2023.kBackLeftDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new CANSparkMax(HardwareIds.OFFSEASON_2023.kBackLeftTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.OFFSEASON_2023.kBackLeftEncoderId);
          }
          case 3 -> {
            this.m_driveMotor =
                new CANSparkMax(HardwareIds.OFFSEASON_2023.kBackRightDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new CANSparkMax(HardwareIds.OFFSEASON_2023.kBackRightTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.OFFSEASON_2023.kBackRightEncoderId);
          }
          default -> throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
        }
      }
      case ROBOT_2024_COMP -> {
        switch (moduleIndex) {
          case 0 -> {
            this.m_driveMotor =
                new CANSparkMax(HardwareIds.COMP_2024.kFrontLeftDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new CANSparkMax(HardwareIds.COMP_2024.kFrontLeftTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.COMP_2024.kFrontLeftEncoderId);
          }
          case 1 -> {
            this.m_driveMotor =
                new CANSparkMax(HardwareIds.COMP_2024.kFrontRightDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new CANSparkMax(HardwareIds.COMP_2024.kFrontRightTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.COMP_2024.kFrontRightEncoderId);
          }
          case 2 -> {
            this.m_driveMotor =
                new CANSparkMax(HardwareIds.COMP_2024.kBackLeftDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new CANSparkMax(HardwareIds.COMP_2024.kBackLeftTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.COMP_2024.kBackLeftEncoderId);
          }
          case 3 -> {
            this.m_driveMotor =
                new CANSparkMax(HardwareIds.COMP_2024.kBackRightDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new CANSparkMax(HardwareIds.COMP_2024.kBackRightTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.COMP_2024.kBackRightEncoderId);
          }
          default -> throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
        }
      }
      default -> throw new RuntimeException("Invalid robot for ModuleIOSparkMax");
    }

    this.m_driveMotor.restoreFactoryDefaults();
    this.m_turnMotor.restoreFactoryDefaults();

    this.m_driveMotor.setCANTimeout(250);
    this.m_turnMotor.setCANTimeout(250);

    this.m_driveEncoder = this.m_driveMotor.getEncoder();
    this.m_turnRelativeEncoder = this.m_turnMotor.getEncoder();
    this.m_turnAbsoluteEncoder = this.m_absoluteEncoder.getAbsolutePosition();

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
        OdometryQueueThread.getInstance().registerSignal(m_driveEncoder::getPosition);
    this.turnPositionQueue =
        OdometryQueueThread.getInstance().registerSignal(m_turnRelativeEncoder::getPosition);

    this.m_driveMotor.burnFlash();
    this.m_turnMotor.burnFlash();

    this.m_absoluteEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(m_driveEncoder.getPosition()) / DriveBase.kDriveGearing;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveEncoder.getVelocity())
            / DriveBase.kDriveGearing;
    inputs.driveAppliedVolts = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {m_driveMotor.getOutputCurrent()};

    // Refresh the Encoder data because it is cached. This is non-blocking.
    m_turnAbsoluteEncoder.refresh();
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(m_turnAbsoluteEncoder.getValueAsDouble());

    inputs.turnPosition =
        Rotation2d.fromRotations(m_turnRelativeEncoder.getPosition() / DriveBase.kTurnGearing);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_turnRelativeEncoder.getVelocity())
            / DriveBase.kTurnGearing;
    inputs.turnAppliedVolts = m_turnMotor.getAppliedOutput() * m_turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {m_turnMotor.getOutputCurrent()};

    inputs.odometryDrivePositionsRad =
        this.drivePositionQueue.stream()
            .map(
                v ->
                    new TimestampedSensorMeasurement<>(
                        v.getTimestampSeconds(), v.getMeasurement() / DriveBase.kDriveGearing))
            .toList();
    inputs.odometryTurnPositions =
        this.turnPositionQueue.stream()
            .map(
                v ->
                    new TimestampedSensorMeasurement<>(
                        v.getTimestampSeconds(),
                        Rotation2d.fromRotations(v.getMeasurement() / DriveBase.kTurnGearing)))
            .toList();

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
