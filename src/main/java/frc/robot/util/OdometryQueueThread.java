package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
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
public class OdometryQueueThread implements AutoCloseable {
  private final List<DoubleSupplier> signals = new ArrayList<>();
  private final List<Queue<TimestampedSensorMeasurement<Double>>> queues = new ArrayList<>();

  private final Notifier notifier;
  private static OdometryQueueThread instance = null;

  public static OdometryQueueThread getInstance() {
    if (instance == null) {
      instance = new OdometryQueueThread();
    }
    return instance;
  }

  private OdometryQueueThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("OdometryQueueThread");
    notifier.startPeriodic(1.0 / PoseEstimator.ODOMETRY_FREQUENCY);
  }

  /**
   * Registers a valid pollable sensor value accessor into the queues of the thread.
   *
   * @param signal sensor signal supplier
   * @return Returns {@link ArrayBlockingQueue} of the timestamp the sensor reading was made, and
   *     the sensor reading.
   */
  public Queue<TimestampedSensorMeasurement<Double>> registerSignal(DoubleSupplier signal) {
    Queue<TimestampedSensorMeasurement<Double>> queue = new ArrayBlockingQueue<>(100);
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
        queues.get(i).offer(new TimestampedSensorMeasurement<>(signals.get(i).getAsDouble()));
      }
    } finally {
      PoseEstimator.odometryLock.unlock();
    }
  }

  @Override
  public void close() {
    notifier.close();
  }
}
