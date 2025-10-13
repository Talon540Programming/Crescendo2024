package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.PigeonConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
// import frc.robot.subsystems.drive.GyroIO.GyroIOData;
// import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(id);

  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> roll = pigeon.getRoll();
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityXWorld();
  private final StatusSignal<AngularVelocity> rollVelocity = pigeon.getAngularVelocityYWorld();

  private final Queue<Double> yawPositionQueue;

  // private final Queue<Double> yawTimestampQueue;

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw.setUpdateFrequency(DriveConstants.odometryFrequencyHz);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
    pigeon.optimizeBusUtilization();

    yawPositionQueue =
        OdometryManager.getInstance().registerSignal(yaw.refresh()::getValueAsDouble);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    inputs.data =
        new GyroIOData(
            BaseStatusSignal.isAllGood(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity),
            Rotation2d.fromDegrees(yaw.getValueAsDouble()),
            Units.degreesToRadians(yawVelocity.getValueAsDouble()),
            Rotation2d.fromDegrees(pitch.getValueAsDouble()),
            Units.degreesToRadians(pitchVelocity.getValueAsDouble()),
            Rotation2d.fromDegrees(roll.getValueAsDouble()),
            Units.degreesToRadians(rollVelocity.getValueAsDouble()));

    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
    // inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) ->
    // value).toArray();
    // yawTimestampQueue.clear();
  }
}
