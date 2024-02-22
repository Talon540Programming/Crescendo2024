package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class ShooterStateStruct implements Struct<ShooterState> {
  @Override
  public Class<ShooterState> getTypeClass() {
    return ShooterState.class;
  }

  @Override
  public String getTypeString() {
    return "struct:ShooterState";
  }

  @Override
  public int getSize() {
    return Rotation2d.struct.getSize() + kSizeDouble;
  }

  @Override
  public String getSchema() {
    return "Rotation2d angle;double velocity";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Rotation2d.struct};
  }

  @Override
  public ShooterState unpack(ByteBuffer byteBuffer) {
    var angle = Rotation2d.struct.unpack(byteBuffer);
    var speed = byteBuffer.getDouble();
    return new ShooterState(angle, speed);
  }

  @Override
  public void pack(ByteBuffer byteBuffer, ShooterState shooterState) {
    Rotation2d.struct.pack(byteBuffer, shooterState.angle());
    byteBuffer.putDouble(shooterState.shooterVelocityMetersPerSecond());
  }
}
