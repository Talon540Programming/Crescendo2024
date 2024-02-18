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
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class PoseEstimator {
  public static final double ODOMETRY_FREQUENCY = 100.0;
  public static final Lock odometryLock = new ReentrantLock();
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.01, 0.01, Math.toRadians(0.5));
  private static final double kBufferDurationSec = 0.5;

  private static PoseEstimator instance;

  private Pose2d pose = new Pose2d();
  private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
  private final TimeInterpolatableBuffer<OdometryUpdate> m_poseBuffer = TimeInterpolatableBuffer.createBuffer(kBufferDurationSec);


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
  }

  public void addDriveData(double timestampSeconds, Twist2d twist) {
    this.pose = pose.exp(twist);
  }

  public void addVisionData(
      double timestampSeconds, Pose2d pose, Matrix<N3, N1> visionMeasurementStdDevs) {

        var sampleOpt = m_poseBuffer.getSample(timestampSeconds);

        if (sampleOpt.isEmpty()) return;

        var sample = sampleOpt.get();
        var samplePose = sample.endPose;
        
      }

  public record OdometryUpdate(Pose2d poseMeter, Twist2d applieTwist2d) implements Interpolatable<OdometryUpdate> {
    @Override
    public OdometryUpdate interpolate(OdometryUpdate endValue, double t) {
      if (t < 0) {
        return this;
      } else if (t >= 1) {
        return endValue;
      } else {

        var poseLerp = endPose.interpolate(endValue.endPose, t);
        var twistLerp = endPose.log(poseLerp);
        return new OdometryUpdate(poseLerp, twistLerp);
      }
    }
  }
}
