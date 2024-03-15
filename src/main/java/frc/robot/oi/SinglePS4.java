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
  public Trigger shooterAngleLock() {
    return controller.R1();
  }

  @Override
  public Trigger trajectoryOverride() {
    return controller.square();
  }

  @Override
  public Trigger shoot() {
    return controller.touchpad();
  }

  @Override
  public Trigger shootPoseOverride() {
    return controller.L2();
  }

  @Override
  public Trigger feederIntakePoseOverride() {
    return controller.L3();
  }
}
