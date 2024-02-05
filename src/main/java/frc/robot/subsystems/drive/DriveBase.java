package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
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
  public static final double kMaxAngularVelocityRadiansPerSecond;

  public static final SwerveDriveKinematics m_kinematics;

  private final GyroIO m_gyroIO;
  private final GyroIOInputs m_gyroInputs = new GyroIOInputs();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR

  private Rotation2d m_lastGyroRotation = new Rotation2d();

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_SIMBOT, ROBOT_2024_COMP -> {
        kDriveGearing = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        kTurnGearing = 12.8;
        kWheelRadiusMeters = Units.inchesToMeters(2.0);
        kTrackWidthXMeters = Units.inchesToMeters(22.5);
        kTrackWidthYMeters = Units.inchesToMeters(22.5);

        kMaxLinearVelocityMetersPerSecond = Units.feetToMeters(14.5);
      }
      default -> throw new RuntimeException("Unknown RobotType for Drivebase");
    }

    kDriveBaseRadiusMeters = Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);
    kMaxAngularVelocityRadiansPerSecond =
        kMaxLinearVelocityMetersPerSecond / kDriveBaseRadiusMeters;
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
    m_gyroIO.updateInputs(m_gyroInputs);
    for (var module : m_modules) {
      module.updateInputs();
    }
    PoseEstimator.odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", m_gyroInputs);
    for (var module : m_modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      // Stop moving when disabled
      for (var module : m_modules) {
        module.disable();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    int deltaCount =
        m_gyroInputs.connected ? m_gyroInputs.odometryYawPositions.size() : Integer.MAX_VALUE;
    for (var module : m_modules) {
      deltaCount = Math.min(deltaCount, module.getPositionDeltas().size());
    }
    for (int i = 0; i < deltaCount; i++) {
      double timestampSeconds = Double.NEGATIVE_INFINITY;
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        var moduleMeasurement = m_modules[j].getPositionDeltas().get(i);
        timestampSeconds = Math.max(moduleMeasurement.getTimestampSeconds(), timestampSeconds);
        wheelDeltas[j] = moduleMeasurement.getMeasurement();
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = m_kinematics.toTwist2d(wheelDeltas);
      if (m_gyroInputs.connected) {
        var gyroMeasurement = m_gyroInputs.odometryYawPositions.get(i);
        timestampSeconds = Math.max(gyroMeasurement.getTimestampSeconds(), timestampSeconds);
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroMeasurement.getMeasurement();

        twist.dtheta = gyroRotation.minus(m_lastGyroRotation).getRadians();

        m_lastGyroRotation = gyroRotation;
      }

      // Apply the twist (change since last sample) to the current pose
      PoseEstimator.getInstance().addDriveData(timestampSeconds, twist);
    }

    Logger.recordOutput("Odometry/EstimatedPose", PoseEstimator.getInstance().getPose());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearVelocityMetersPerSecond);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = m_modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
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

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  public void runCharacterizationVolts(double volts) {
    for (var module : m_modules) {
      module.runCharacterization(volts);
    }
  }

  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : m_modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  public static Translation2d[] getModuleTranslations(double trackWidthX, double trackWidthY) {
    return new Translation2d[] {
      new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)
    };
  }
}
