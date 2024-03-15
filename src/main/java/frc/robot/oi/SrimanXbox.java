package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.CommandSrimanController;

public class SrimanXbox implements ControlsInterface {
  private final CommandSrimanController controller = new CommandSrimanController(0);

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
  public Trigger shooterAngleLock() {
    return controller.rightBumper();
  }

  @Override
  public Trigger trajectoryOverride() {
    return controller.y();
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
