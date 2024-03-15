package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.Constants;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public class OdometryQueueManager implements AutoCloseable {
  private static final int NUMBER_OF_SAMPLES = 20;
  private static OdometryQueueManager instance;

  public static OdometryQueueManager getInstance() {
    if (instance == null) {
      instance = new OdometryQueueManager();
    }
    return instance;
  }

  private final Notifier notifier;

  private final List<ModuleOdometrySuppliers> odometrySuppliers = new ArrayList<>(4);
  private Supplier<Optional<Rotation2d>> gyroAngleSupplier;

  private final List<Queue<SwerveModulePosition>> odometryQueues = new ArrayList<>(4);
  private Queue<Rotation2d> gyroAngleQueue;
  private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(NUMBER_OF_SAMPLES);

  public OdometryQueueManager() {
    notifier = new Notifier(this::periodic);
    notifier.setName("OdometryQueueThread");
    notifier.startPeriodic(1.0 / PoseEstimator.ODOMETRY_FREQUENCY);

    // Sim doesn't have a gyro, use this as a fake
    if (Constants.getRobotMode() == Constants.RobotMode.REAL) {
      registerGyro(() -> Optional.of(new Rotation2d()));
    }
  }

  private void periodic() {
    PoseEstimator.odometryLock.lock();
    try {
      if (gyroAngleSupplier == null || odometrySuppliers.size() != 4) {
        return;
      }

      double timestamp = MathSharedStore.getTimestamp();
      var gyroAngleOpt = gyroAngleSupplier.get();

      if (gyroAngleOpt.isEmpty()) {
        return;
      }

      for (int i = 0; i < 4; i++) {
        var odometrySupplier = odometrySuppliers.get(i);
        var wheelPositionMetersOpt = odometrySupplier.drivePositionMetersSupplier.get();
        var moduleAngleOpt = odometrySupplier.angleSupplier.get();

        if (wheelPositionMetersOpt.isEmpty() || moduleAngleOpt.isEmpty()) {
          return;
        }

        odometryQueues
            .get(i)
            .offer(
                new SwerveModulePosition(
                    wheelPositionMetersOpt.getAsDouble(), moduleAngleOpt.get()));
      }

      timestampQueue.offer(timestamp);
      gyroAngleQueue.offer(gyroAngleOpt.get());
    } finally {
      PoseEstimator.odometryLock.unlock();
    }
  }

  public Queue<SwerveModulePosition> registerModule(
      Supplier<OptionalDouble> drivePositionMetersSupplier,
      Supplier<Optional<Rotation2d>> angleSupplier) {
    var queue = new ArrayBlockingQueue<SwerveModulePosition>(NUMBER_OF_SAMPLES);
    PoseEstimator.odometryLock.lock();
    try {
      odometrySuppliers.add(
          new ModuleOdometrySuppliers(drivePositionMetersSupplier, angleSupplier));
      odometryQueues.add(queue);
    } finally {
      PoseEstimator.odometryLock.unlock();
    }

    return queue;
  }

  public Queue<Rotation2d> registerGyro(Supplier<Optional<Rotation2d>> yawSupplier) {
    var queue = new ArrayBlockingQueue<Rotation2d>(NUMBER_OF_SAMPLES);
    PoseEstimator.odometryLock.lock();
    try {
      gyroAngleSupplier = yawSupplier;
      gyroAngleQueue = queue;
    } finally {
      PoseEstimator.odometryLock.unlock();
    }

    return queue;
  }

  public void updateTimestampsInput(OdometryTimestampInputs inputs) {
    // Add a fake time sample for SIM
    if (Constants.getRobotMode() == Constants.RobotMode.SIM) {
      inputs.timestamps = new double[] {MathSharedStore.getTimestamp()};
    } else {
      inputs.timestamps = timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    }

    inputs.sampleCount = inputs.timestamps.length;
    timestampQueue.clear();
  }

  @Override
  public void close() {
    notifier.close();
  }

  private record ModuleOdometrySuppliers(
      Supplier<OptionalDouble> drivePositionMetersSupplier,
      Supplier<Optional<Rotation2d>> angleSupplier) {}

  @AutoLog
  public static class OdometryTimestampInputs {
    public int sampleCount;
    public double[] timestamps;
  }
}
