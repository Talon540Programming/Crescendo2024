package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.constants.Constants;
import frc.robot.util.OdometryQueueManager;
import frc.robot.util.OdometryTimestampInputsAutoLogged;
import frc.robot.util.PoseEstimator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveBase extends SubsystemBase {
  public static final double kDriveGearing;
  public static final double kTurnGearing;
  public static final double kWheelRadiusMeters;

  public static final double kTrackWidthXMeters;
  public static final double kTrackWidthYMeters;
  public static final double kDriveBaseRadiusMeters;

  public static final double kMaxLinearVelocityMetersPerSecond;
  public static final double kMaxLinearAccelerationMetersPerSecondSquared;
  public static final double kMaxAngularVelocityRadiansPerSecond;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared;

  private static final SwerveDriveKinematics m_kinematics;

  private final GyroIO m_gyroIO;
  private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final OdometryTimestampInputsAutoLogged m_odometryTimestampInputs =
      new OdometryTimestampInputsAutoLogged();

  private Rotation2d m_lastGyroRotation = new Rotation2d();

  private final SwerveModulePosition[] m_lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_SIMBOT, ROBOT_2024_COMP -> {
        kDriveGearing = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        kTurnGearing = 12.8;
        kWheelRadiusMeters = Units.inchesToMeters(2.0);
        kTrackWidthXMeters = Units.inchesToMeters(22.5);
        kTrackWidthYMeters = Units.inchesToMeters(22.5);

        kMaxLinearVelocityMetersPerSecond = Units.feetToMeters(14.5);
        kMaxLinearAccelerationMetersPerSecondSquared = Units.feetToMeters(25.0);
      }
      default -> throw new RuntimeException("Unknown RobotType for Drivebase");
    }

    kDriveBaseRadiusMeters = Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);
    kMaxAngularVelocityRadiansPerSecond =
        kMaxLinearVelocityMetersPerSecond / kDriveBaseRadiusMeters;
    kMaxAngularAccelerationRadiansPerSecondSquared =
        kMaxLinearAccelerationMetersPerSecondSquared / kDriveBaseRadiusMeters;

    m_kinematics =
        new SwerveDriveKinematics(getModuleTranslations(kTrackWidthXMeters, kTrackWidthYMeters));
  }

  public DriveBase(
      GyroIO gyroIO,
      ModuleIO frontLeftIO,
      ModuleIO frontRightIO,
      ModuleIO backLeftIO,
      ModuleIO backRightIO) {
    this.m_gyroIO = gyroIO;

    m_modules[0] = new Module(0, frontLeftIO);
    m_modules[1] = new Module(1, frontRightIO);
    m_modules[2] = new Module(2, backLeftIO);
    m_modules[3] = new Module(3, backRightIO);
  }

  public void periodic() {
    PoseEstimator.odometryLock.lock();
    try {
      OdometryQueueManager.getInstance().updateTimestampsInput(m_odometryTimestampInputs);
      m_gyroIO.updateInputs(m_gyroInputs);
      for (var module : m_modules) {
        module.updateInputs();
      }
    } finally {
      PoseEstimator.odometryLock.unlock();
    }

    // We don't want to block the odometry thread unless we need to. This ensures we collect the
    // most number of samples possible.
    Logger.processInputs("Odometry/Timestamps", m_odometryTimestampInputs);
    Logger.processInputs("Drive/Gyro", m_gyroInputs);
    for (var module : m_modules) {
      module.processInputs();
    }

    if (DriverStation.isDisabled()) {
      // Reset the setpoints when the robot is disabled
      for (var module : m_modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("Drive/ModuleSetpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/ModuleSetpointsOptimized", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SpeedSetpoint", new ChassisSpeeds());
    }

    // Update PoseEstimator with odometry deltas
    for (int sampleIndex = 0; sampleIndex < m_odometryTimestampInputs.sampleCount; sampleIndex++) {
      double timestampSeconds = m_odometryTimestampInputs.timestamps[sampleIndex];
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        var positionMeasurement = m_modules[moduleIndex].getModuleDeltas()[sampleIndex];
        wheelDeltas[moduleIndex] =
            new SwerveModulePosition(
                positionMeasurement.distanceMeters
                    - m_lastModulePositions[moduleIndex].distanceMeters,
                positionMeasurement.angle);
        m_lastModulePositions[moduleIndex] = positionMeasurement;
      }

      var deltaTwist = m_kinematics.toTwist2d(wheelDeltas);

      // Use the gyro if possible
      if (m_gyroInputs.connected) {
        var yawMeasurement = m_gyroInputs.odometryYawPositions[sampleIndex];
        // Replace the delta theta component from the one from the gyro
        deltaTwist.dtheta = yawMeasurement.minus(m_lastGyroRotation).getRadians();
        m_lastGyroRotation = yawMeasurement;
      }

      PoseEstimator.getInstance().addDriveData(timestampSeconds, deltaTwist);
    }

    Logger.recordOutput("Odometry/EstimatedPose", PoseEstimator.getInstance().getPose());
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "Drive/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getCurrentState();
    }
    return states;
  }

  public static Translation2d[] getModuleTranslations(double trackWidthX, double trackWidthY) {
    return new Translation2d[] {
      new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)
    };
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearVelocityMetersPerSecond);

    // Apply setpoints and form optimized states
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      optimizedStates[i] = m_modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("Drive/SpeedSetpoint", discreteSpeeds);
    Logger.recordOutput("Drive/ModuleSetpoints", setpointStates);
    Logger.recordOutput("Drive/ModuleSetpointsOptimized", optimizedStates);
  }

  public Command getDriveCharacterizationCommand() {
    return new FeedForwardCharacterization(
        this,
        "Drive",
        (var voltage) -> {
          for (var module : m_modules) {
            module.runCharacterizationVoltage(Rotation2d.fromDegrees(0), voltage);
          }
        },
        () -> {
          double sum = 0;
          for (var module : m_modules) {
            sum += module.getCharacterizationVelocity();
          }

          return sum / 4.0;
        },
        0.5,
        15);
  }

  /**
   * Get the current velocity of the drivetrain
   *
   * @return current velocity as a ChassisSpeeds object
   */
  public ChassisSpeeds getVelocity() {
    var speeds = m_kinematics.toChassisSpeeds(getModuleStates());
    if (m_gyroInputs.connected) {
      speeds.omegaRadiansPerSecond = m_gyroInputs.yawVelocityRadPerSec;
    }

    return speeds;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    var translations = getModuleTranslations(kTrackWidthXMeters, kTrackWidthYMeters);
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = translations[i].getAngle();
    }
    m_kinematics.resetHeadings(headings);
    stop();
  }
}
