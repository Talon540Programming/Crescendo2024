package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class PoseEstimator {
  private static PoseEstimator instance;

  public static final double ODOMETRY_FREQUENCY = 100.0;
  public static final Lock odometryLock = new ReentrantLock();

  private Pose2d pose = new Pose2d();

  private PoseEstimator() {}

  public static PoseEstimator getInstance() {
    if (instance == null) {
      instance = new PoseEstimator();
    }
    return instance;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void resetPose(Pose2d pose) {
    this.pose = pose;
  }

  public void addDriveData(Twist2d twist) {
    this.pose = pose.exp(twist);
  }
}
