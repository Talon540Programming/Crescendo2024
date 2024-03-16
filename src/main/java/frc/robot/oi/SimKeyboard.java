package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimKeyboard implements ControlsInterface {
  private final GenericHID hid = new GenericHID(0);

  @Override
  public double getDriveX() {
    return -hid.getRawAxis(1);
  }

  @Override
  public double getDriveY() {
    return -hid.getRawAxis(0);
  }

  @Override
  public double getDriveTheta() {
    return -hid.getRawAxis(2);
  }

  @Override
  public Trigger robotRelativeOverride() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger moduleLock() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger headingLock() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger trajectoryOverride() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger shoot() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger intake() {
    return new Trigger(() -> false);
  }
}
