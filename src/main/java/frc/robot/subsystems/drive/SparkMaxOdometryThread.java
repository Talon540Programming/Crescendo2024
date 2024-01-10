package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.util.PoseEstimator;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class SparkMaxOdometryThread {
  private final List<DoubleSupplier> signals = new ArrayList<>();
  private final List<Queue<Double>> queues = new ArrayList<>();

  private final Notifier notifier;
  private static SparkMaxOdometryThread instance = null;

  public static SparkMaxOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkMaxOdometryThread();
    }
    return instance;
  }

  private SparkMaxOdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkMaxOdometryThread");
    notifier.startPeriodic(1.0 / PoseEstimator.ODOMETRY_FREQUENCY);
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(100);
    PoseEstimator.odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      PoseEstimator.odometryLock.unlock();
    }
    return queue;
  }

  private void periodic() {
    PoseEstimator.odometryLock.lock();
    try {
      for (int i = 0; i < signals.size(); i++) {
        queues.get(i).offer(signals.get(i).getAsDouble());
      }
    } finally {
      PoseEstimator.odometryLock.unlock();
    }
  }
}
