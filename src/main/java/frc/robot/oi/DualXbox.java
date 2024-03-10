package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DualXbox implements ControlsInterface {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  @Override
  public double getDriveX() {
    return -driverController.getLeftY();
  }

  @Override
  public double getDriveY() {
    return -driverController.getLeftX();
  }

  @Override
  public double getDriveTheta() {
    return -driverController.getRightX();
  }

  @Override
  public Trigger robotRelativeOverride() {
    return driverController.leftBumper();
  }

  @Override
  public Trigger moduleLock() {
    return driverController.x();
  }

  @Override
  public Trigger shooterAngleLock() {
    return driverController.rightBumper();
  }

  @Override
  public Trigger trajectoryOverride() {
    return driverController.y();
  }

  @Override
  public Trigger shoot() {
    return operatorController.x();
  }
}
