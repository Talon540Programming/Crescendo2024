package frc.robot.util;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class SparkMaxUtils {
  /**
   * Configures SparkMax(s) configured to follow the output of another SparkMax but still reports
   * normal telemetry data.
   *
   * @param motors motors to configure
   */
  public static void configureFollowers(CANSparkMax... motors) {
    for (var motor : motors) {
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
    }
  }

  /**
   * Disables non-critical telemetry data for encoder ports of the SparkMax(s). This can reduce CAN
   * utilization if these ports aren't used. StatusFrames are found <a
   * href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">here</a>
   *
   * @param motors motors to configure
   */
  public static void disableSensorFrames(CANSparkMax... motors) {
    for (var motor : motors) {
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 65535);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 65535);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 65535);
      motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 65535);
    }
  }
}
