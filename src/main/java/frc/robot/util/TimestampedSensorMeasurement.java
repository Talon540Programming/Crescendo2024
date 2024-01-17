package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;

public class TimestampedSensorMeasurement<T> {
  private final double timestampSeconds;
  private final T measurementValue;

  public TimestampedSensorMeasurement(double timestampSeconds, T measurementValue) {
    this.timestampSeconds = timestampSeconds;
    this.measurementValue = measurementValue;
  }

  public TimestampedSensorMeasurement(T measurementValue) {
    this(MathSharedStore.getTimestamp(), measurementValue);
  }

  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  public T getMeasurement() {
    return measurementValue;
  }
}
