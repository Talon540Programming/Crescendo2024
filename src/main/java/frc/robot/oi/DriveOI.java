package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriveOI {
  public double getDriveX();

  public double getDriveY();

  public double getDriveTheta();

  public Trigger robotRelativeOverride();

  public Trigger moduleLock();

  public Trigger headingLock();

  public Trigger subwooferPoseOverride();

  public Trigger sourcePoseOverride();
}
