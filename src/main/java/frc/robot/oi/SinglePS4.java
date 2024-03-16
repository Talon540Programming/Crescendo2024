package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SinglePS4 implements ControlsInterface {
  private final CommandPS4Controller controller = new CommandPS4Controller(0);

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
    return controller.L1();
  }

  @Override
  public Trigger moduleLock() {
    return controller.cross();
  }

  @Override
  public Trigger headingLock() {
    return controller.R1();
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
    return controller.touchpad();
  }

  @Override
  public Trigger intake() {
    return controller.cross();
  }

  @Override
  public Trigger ejectIndexer() {
    return controller.triangle();
  }
}
