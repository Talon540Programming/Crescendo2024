package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareIds;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 m_gyro;

  private final StatusSignal<Angle> m_roll;
  private final StatusSignal<Angle> m_pitch;
  private final StatusSignal<Angle> m_yaw;

  private final StatusSignal<AngularVelocity> m_rollVelocity;
  private final StatusSignal<AngularVelocity> m_pitchVelocity;
  private final StatusSignal<AngularVelocity> m_yawVelocity;

  // private final StatusSignal<Double> m_accelX;
  // private final StatusSignal<Double> m_accelY;
  // private final StatusSignal<Double> m_accelZ;

  public GyroIOPigeon2() {
    switch (Constants.getRobotType()) {
      case ROBOT_2024_COMP -> this.m_gyro = new Pigeon2(HardwareIds.COMP_2024.kPigeonId);
      default -> throw new RuntimeException("Invalid RobotType for GyroIOPigeon2");
    }

    this.m_gyro.getConfigurator().setYaw(0.0);

    this.m_roll = this.m_gyro.getRoll();
    this.m_pitch = this.m_gyro.getPitch();
    this.m_yaw = this.m_gyro.getYaw();

    this.m_rollVelocity = this.m_gyro.getAngularVelocityXWorld();
    this.m_pitchVelocity = this.m_gyro.getAngularVelocityYWorld();
    this.m_yawVelocity = this.m_gyro.getAngularVelocityZWorld();

    // this.m_accelX = this.m_gyro.getAccelerationX();
    // this.m_accelY = this.m_gyro.getAccelerationY();
    // this.m_accelZ = this.m_gyro.getAccelerationZ();

    // Faster rate for Yaw for Odometry
    this.m_yaw.setUpdateFrequency(100);
    this.m_yawVelocity.setUpdateFrequency(100);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, m_roll, m_pitch, m_rollVelocity, m_pitchVelocity);

    m_gyro.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Only check yaw and yaw velocity as they are needed for odometry
    inputs.connected =
        BaseStatusSignal.refreshAll(
                m_roll,
                m_pitch,
                m_yaw,
                m_rollVelocity,
                m_pitchVelocity,
                m_yawVelocity)
            .equals(StatusCode.OK);

    inputs.rollPosition = Rotation2d.fromDegrees(m_roll.getValueAsDouble());
    inputs.pitchPosition = Rotation2d.fromDegrees(m_pitch.getValueAsDouble());
    inputs.yawPosition = Rotation2d.fromDegrees(m_yaw.getValueAsDouble());

    inputs.rollVelocityRadPerSec = Units.degreesToRadians(m_rollVelocity.getValueAsDouble());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(m_pitchVelocity.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_yawVelocity.getValueAsDouble());
  }
}
