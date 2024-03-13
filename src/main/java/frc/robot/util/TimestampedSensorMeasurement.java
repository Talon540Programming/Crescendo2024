package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import lombok.Getter;

public class TimestampedSensorMeasurement<T> {
  @Getter private final double timestampSeconds;
  private final T measurementValue;

  public TimestampedSensorMeasurement(double timestampSeconds, T measurementValue) {
    this.timestampSeconds = timestampSeconds;
    this.measurementValue = measurementValue;
  }

  public TimestampedSensorMeasurement(T measurementValue) {
    this(MathSharedStore.getTimestamp(), measurementValue);
  }

  public T getMeasurement() {
    return measurementValue;
  }
}
