package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.PoseEstimator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveBase extends SubsystemBase {
  private final GyroIO m_gyroIO;
  private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR

  private Rotation2d m_lastGyroRotation = new Rotation2d();

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
        m_gyroInputs.connected ? m_gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    for (var module : m_modules) {
      deltaCount = Math.min(deltaCount, module.getPositionDeltas().length);
    }
    for (int i = 0; i < deltaCount; i++) {
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelDeltas[j] = m_modules[j].getPositionDeltas()[i];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = Constants.Drivetrain.m_kinematics.toTwist2d(wheelDeltas);
      if (m_gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = m_gyroInputs.odometryYawPositions[i];
        twist.dtheta = gyroRotation.minus(m_lastGyroRotation).getRadians();

        m_lastGyroRotation = gyroRotation;
      }

      // Apply the twist (change since last sample) to the current pose
      PoseEstimator.getInstance().addDriveData(twist);
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
    SwerveModuleState[] setpointStates =
        Constants.Drivetrain.m_kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond);

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
    var translations = Constants.Drivetrain.getModuleTranslations();
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = translations[i].getAngle();
    }
    Constants.Drivetrain.m_kinematics.resetHeadings(headings);
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
}
