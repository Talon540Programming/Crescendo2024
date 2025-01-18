package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareIds;

public class ModuleIOSparkMax implements ModuleIO {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;
  private final CANcoder m_absoluteEncoder;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;
  private final StatusSignal<Angle> m_turnAbsoluteEncoder;


  public ModuleIOSparkMax(int moduleIndex) {
    switch (Constants.getRobotType()) {
      case ROBOT_2024_COMP -> {
        switch (moduleIndex) {
          case 0 -> {
            this.m_driveMotor =
                new SparkMax(HardwareIds.COMP_2024.kFrontLeftDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new SparkMax(HardwareIds.COMP_2024.kFrontLeftTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.COMP_2024.kFrontLeftEncoderId);
          }
          case 1 -> {
            this.m_driveMotor =
                new SparkMax(HardwareIds.COMP_2024.kFrontRightDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new SparkMax(HardwareIds.COMP_2024.kFrontRightTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.COMP_2024.kFrontRightEncoderId);
          }
          case 2 -> {
            this.m_driveMotor =
                new SparkMax(HardwareIds.COMP_2024.kBackLeftDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new SparkMax(HardwareIds.COMP_2024.kBackLeftTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.COMP_2024.kBackLeftEncoderId);
          }
          case 3 -> {
            this.m_driveMotor =
                new SparkMax(HardwareIds.COMP_2024.kBackRightDriveId, MotorType.kBrushless);
            this.m_turnMotor =
                new SparkMax(HardwareIds.COMP_2024.kBackRightTurnId, MotorType.kBrushless);
            this.m_absoluteEncoder = new CANcoder(HardwareIds.COMP_2024.kBackRightEncoderId);
          }
          default -> throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
        }
      }
      default -> throw new RuntimeException("Invalid robot for ModuleIOSparkMax");
    }

    this.m_driveEncoder = this.m_driveMotor.getEncoder();
    this.m_turnRelativeEncoder = this.m_turnMotor.getEncoder();
    this.m_turnAbsoluteEncoder = this.m_absoluteEncoder.getAbsolutePosition();

    this.m_driveEncoder.setPosition(0.0);

    this.m_turnRelativeEncoder.setPosition(0.0);

    this.m_turnAbsoluteEncoder.setUpdateFrequency(50);
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
  }

  @Override
  public void setDriveVoltage(double volts) {
    System.out.println("DRIVE VOLTAGE CHECKPOINT");
    m_driveMotor.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnMotor.setVoltage(volts);
    System.out.println("TURN VOLTAGE CHECKPOINT");
    System.out.println(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    m_driveMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    m_turnMotor.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
