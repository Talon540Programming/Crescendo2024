package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleXbox implements ControlsInterface {
  private final CommandXboxController controller = new CommandXboxController(0);

  @Override
  public double getDriveX() {
    return -controller.getLeftY();
  }

  @Override
  public double getDriveY() {
    return -controller.getLeftX();
  }

  @Override
  public double getDriveTheta() {
    return -controller.getRightX();
  }

  @Override
  public Trigger robotRelativeOverride() {
    return controller.leftBumper();
  }

  @Override
  public Trigger moduleLock() {
    return controller.x();
  }

  @Override
  public Trigger headingLock() {
    return controller.rightBumper();
  }

  @Override
  public Trigger subwooferPoseOverride() {
    return controller.povUp();
  }

  @Override
  public Trigger sourcePoseOverride() {
    return controller.povDown();
  }

  @Override
  public Trigger shoot() {
    return controller.rightTrigger();
  }

  @Override
  public Trigger intake() {
    return controller.leftTrigger();
  }
}
