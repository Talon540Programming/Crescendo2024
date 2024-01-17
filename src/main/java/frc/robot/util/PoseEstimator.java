package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class PoseEstimator {
  private static final double kBufferDurationSec = 0.5;
  public static final double ODOMETRY_FREQUENCY = 100.0;
  public static final Lock odometryLock = new ReentrantLock();

  private static PoseEstimator instance;

  private Pose2d pose = new Pose2d();
  private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
  private final TimeInterpolatableBuffer<Pose2d> m_poseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferDurationSec);

  public static PoseEstimator getInstance() {
    if (instance == null) {
      instance = new PoseEstimator();
    }
    return instance;
  }

  private PoseEstimator() {
    Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.003, 0.003, 0.0002);
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
  }

  public void addVisionData(
      double timestampSeconds, Pose2d pose, Matrix<N3, N1> visionMeasurementStdDevs) {}
}
