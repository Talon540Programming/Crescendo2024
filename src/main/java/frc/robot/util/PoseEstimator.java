package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.NoSuchElementException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class PoseEstimator {
  // Standard deviations of the pose estimate (x position in meters, y position in meters, and
  // heading in radians).
  // Increase these numbers to trust your state estimate less.
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.003, 0.003, 0.0002); // TODO
  public static final double ODOMETRY_FREQUENCY = 100.0;
  public static final Lock odometryLock = new ReentrantLock();

  private static PoseEstimator instance;

  private Pose2d pose = new Pose2d();
  private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
  private static final double kBufferDurationSec = 0.3;
  private final TimeInterpolatableBuffer<OdometryUpdate> m_poseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferDurationSec);

  public static PoseEstimator getInstance() {
    if (instance == null) {
      instance = new PoseEstimator();
    }
    return instance;
  }

  private PoseEstimator() {
    for (int i = 0; i < 3; ++i) {
      m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }
  }

  public Pose2d getPose() {
    return pose;
  }

  public void resetPose(Pose2d pose) {
    this.pose = pose;
    m_poseBuffer.clear();
  }

  public void addDriveData(double timestampSeconds, Twist2d twist) {
    this.pose = pose.exp(twist);
    m_poseBuffer.addSample(timestampSeconds, new OdometryUpdate(this.pose, twist));
  }

  public void addVisionData(
      double timestampSeconds, Pose2d pose, Matrix<N3, N1> visionMeasurementStdDevs) {
    // If this measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (m_poseBuffer.getInternalBuffer().lastKey() - kBufferDurationSec > timestampSeconds) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    var sampleOpt = m_poseBuffer.getSample(timestampSeconds);

    if (sampleOpt.isEmpty()) return;

    // Determined position of the robot at the time of the vision update
    var sample = sampleOpt.get();
    var samplePose = sample.pose;

    // Create Kalman gain matrix of trust of vision measurements based on vision measurement std
    // devs
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I
    for (int row = 0; row < 3; ++row) {
      if (m_q.get(row, 0) == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(
            row, row, m_q.get(row, 0) / (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row])));
      }
    }

    // Form the twist between the sample pose and estimated vision pose
    var visionTwist = samplePose.log(pose);

    // Scale the twist by the gain matrix observing bias between current pose and vision pose
    var scaledVecTwist =
        visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));
    var scaledTwist =
        new Twist2d(scaledVecTwist.get(0, 0), scaledVecTwist.get(1, 0), scaledVecTwist.get(2, 0));

    // Reset Odometry to state at sample with vision adjustment.
    resetPose(samplePose.exp(scaledTwist));

    // Record the current pose to allow multiple measurements from the same timestamp
    m_poseBuffer.addSample(timestampSeconds, new OdometryUpdate(this.pose, sample.lerp()));

    // Replay odometry inputs between sample time and latest recorded sample to update the
    // pose buffer and correct odometry
    for (var entry : m_poseBuffer.getInternalBuffer().tailMap(timestampSeconds).entrySet()) {
      addDriveData(entry.getKey(), entry.getValue().lerp());
    }
  }

  /**
   * Represents an Odometry update at a given moment
   *
   * @param pose The pose observed given the current sensor inputs and the previous pose.
   * @param lerp The change measured by the sensors between the previous update and the update's
   *     pose.
   */
  private record OdometryUpdate(Pose2d pose, Twist2d lerp)
      implements Interpolatable<OdometryUpdate> {
    @Override
    public OdometryUpdate interpolate(OdometryUpdate endValue, double t) {
      if (t < 0) {
        return this;
      } else if (t >= 1) {
        return endValue;
      } else {
        var interpolatedPose = pose().interpolate(endValue.pose(), t);
        var lerp = pose().log(interpolatedPose);

        return new OdometryUpdate(interpolatedPose, lerp);
      }
    }
  }
}
