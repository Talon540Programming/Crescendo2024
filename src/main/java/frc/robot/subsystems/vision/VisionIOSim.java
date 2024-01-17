package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import frc.robot.util.PoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim extends VisionIOPhotonCamera {
  private static VisionSystemSim m_visionSystemSim;

  static {
    if (Constants.getRobotMode() == Constants.RobotMode.SIM) {
      m_visionSystemSim = new VisionSystemSim("main");
      m_visionSystemSim.addAprilTags(VisionBase.m_fieldLayout);
    }
  }

  public VisionIOSim(String cameraName, Transform3d robotToCamera) {
    super(cameraName, robotToCamera);
    // All SIM cameras share the same properties except their placements in the RCS
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(50);
    cameraProp.setAvgLatencyMs(25);
    cameraProp.setLatencyStdDevMs(10);

    var camSim = new PhotonCameraSim(this.m_camera, cameraProp);

    camSim.enableRawStream(false);
    camSim.enableProcessedStream(false);
    camSim.enableDrawWireframe(false);

    m_visionSystemSim.addCamera(camSim, this.kRobotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    m_visionSystemSim.update(PoseEstimator.getInstance().getPose());

    super.updateInputs(inputs);

    if (inputs.hasResult) {
      m_visionSystemSim
          .getDebugField()
          .getObject("VisionEstimation")
          .setPose(inputs.estimatedRobotPose.toPose2d());
    } else {
      m_visionSystemSim.getDebugField().getObject("VisionEstimation").setPoses();
    }
  }
}
