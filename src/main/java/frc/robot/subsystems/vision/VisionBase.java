package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.PoseEstimator;
import org.littletonrobotics.junction.Logger;

public class VisionBase extends SubsystemBase {
  public static final AprilTagFieldLayout m_fieldLayout;

  static {
    m_fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  private final VisionIO[] m_cameras;
  private final VisionIOInputs[] m_cameraInputs;

  public VisionBase(VisionIO... cameras) {
    m_cameras = cameras;
    m_cameraInputs = new VisionIOInputs[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      m_cameraInputs[i] = new VisionIOInputs();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_cameras.length; i++) {
      var input = m_cameraInputs[i];

      m_cameras[i].updateInputs(input);
      Logger.processInputs("Vision/Cam" + i, input);

      // Don't report if no tags were found
      if (!input.hasResult) continue;

      PoseEstimator.getInstance()
          .addVisionData(
              input.timestampSeconds,
              input.estimatedRobotPose.toPose2d(),
              input.visionMeasurementStdDevs);
    }
  }
}
