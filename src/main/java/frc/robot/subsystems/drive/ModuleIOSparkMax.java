package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareIds;
import frc.robot.util.OdometryQueueManager;
import frc.robot.util.PoseEstimator;
import frc.robot.util.SparkMaxUtils;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Queue;

public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;
  private final CANcoder m_absoluteEncoder;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;
  private final StatusSignal<Double> m_turnAbsoluteEncoder;

  private final SparkPIDController m_driveController;
  private final SparkPIDController m_turnController;

  private final Queue<SwerveModulePosition> odometryPositionQueue;

  private boolean hasResetPosition = false;

  public ModuleIOSparkMax(int moduleIndex) {
    switch (Constants.getRobotType()) {
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

    this.m_driveMotor.setSmartCurrentLimit(60);
    this.m_turnMotor.setSmartCurrentLimit(60);
    this.m_driveMotor.enableVoltageCompensation(12.0);
    this.m_turnMotor.enableVoltageCompensation(12.0);

    this.m_driveEncoder.setPosition(0.0);
    this.m_driveEncoder.setMeasurementPeriod(10);
    this.m_driveEncoder.setAverageDepth(2);
    // Scale from rotations of the motor shaft to radians of the wheel
    this.m_driveEncoder.setPositionConversionFactor(2 * Math.PI / DriveBase.kDriveGearing);
    // Scale from rotations per minute of the motor shaft to radians per second of the wheel
    this.m_driveEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0 / DriveBase.kDriveGearing);

    this.m_turnRelativeEncoder.setPosition(0.0);
    this.m_turnRelativeEncoder.setMeasurementPeriod(10);
    this.m_turnRelativeEncoder.setAverageDepth(2);
    // Scale from rotations of the motor shaft to radians of the module's angle
    this.m_turnRelativeEncoder.setPositionConversionFactor(2 * Math.PI / DriveBase.kTurnGearing);
    // Scale from rotations per minute of the motor shaft to radians per second of the module's turn
    // rate
    this.m_turnRelativeEncoder.setVelocityConversionFactor(
        (2 * Math.PI) / 60.0 / DriveBase.kTurnGearing);

    this.m_driveController = m_driveMotor.getPIDController();
    this.m_turnController = m_turnMotor.getPIDController();

    this.m_driveMotor.setCANTimeout(0);
    this.m_turnMotor.setCANTimeout(0);

    this.m_driveMotor.burnFlash();
    this.m_turnMotor.burnFlash();

    this.m_driveMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / PoseEstimator.ODOMETRY_FREQUENCY));
    this.m_turnMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / PoseEstimator.ODOMETRY_FREQUENCY));
    SparkMaxUtils.disableSensorFrames(m_driveMotor, m_turnMotor);

    this.m_turnAbsoluteEncoder.setUpdateFrequency(50);
    this.m_absoluteEncoder.optimizeBusUtilization();

    this.odometryPositionQueue =
        OdometryQueueManager.getInstance()
            .registerModule(
                () ->
                    // Convert from rads to meters
                    OptionalDouble.of(m_driveEncoder.getPosition() * DriveBase.kWheelRadiusMeters),
                () ->
                    Optional.of(
                        Rotation2d.fromRotations(
                            m_turnAbsoluteEncoder.refresh().getValueAsDouble())));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = m_driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = m_driveEncoder.getVelocity();
    inputs.driveAppliedVolts = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {m_driveMotor.getOutputCurrent()};

    inputs.turnPositionRad = Rotation2d.fromRadians(m_turnRelativeEncoder.getPosition());
    inputs.turnVelocityRadPerSec = m_turnRelativeEncoder.getVelocity();
    inputs.turnAppliedVolts = m_turnMotor.getAppliedOutput() * m_turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {m_turnMotor.getOutputCurrent()};

    // Refresh the result of the encoder status signal. This is non-blocking.
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(m_turnAbsoluteEncoder.refresh().getValueAsDouble());

    inputs.odometryPositions = this.odometryPositionQueue.toArray(SwerveModulePosition[]::new);
    this.odometryPositionQueue.clear();
  }

  @Override
  public void runDriveVolts(double volts) {
    m_driveMotor.setVoltage(volts);
  }

  @Override
  public void runTurnVolts(double volts) {
    m_turnMotor.setVoltage(volts);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    m_driveController.setP(kP);
    m_driveController.setI(kI);
    m_driveController.setD(kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    m_turnController.setP(kP);
    m_turnController.setI(kI);
    m_turnController.setD(kD);
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadPerSec, double driveFeedForwardVolts) {
    m_driveController.setReference(
        velocityRadPerSec,
        CANSparkBase.ControlType.kVelocity,
        0,
        driveFeedForwardVolts,
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void runTurnAngleSetpoint(Rotation2d angle) {
    if (!hasResetPosition) {
      m_turnRelativeEncoder.setPosition(
          Units.rotationsToRadians(m_turnAbsoluteEncoder.refresh().getValueAsDouble()));
      hasResetPosition = true;
    }

    m_turnController.setReference(
        // Convert from module relative to motor relative
        angle.getRadians(), CANSparkBase.ControlType.kPosition);
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
