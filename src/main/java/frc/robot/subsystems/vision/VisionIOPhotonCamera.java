package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonCamera implements VisionIO {
  protected final PhotonCamera m_camera;
  protected final Transform3d kRobotToCamera;

  private static final Matrix<N3, N1> SINGLE_TARGET_STDDEVS =
      MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, Math.toRadians(10.0)); // TODO
  private static final Matrix<N3, N1> MULTI_TARGET_STDDEVS =
      MatBuilder.fill(Nat.N3(), Nat.N1(), 0.01, 0.01, Math.toRadians(1.0)); // TODO

  public VisionIOPhotonCamera(String cameraName, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(cameraName);
    kRobotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var res = m_camera.getLatestResult();
    inputs.timestampSeconds = res.getTimestampSeconds();
    inputs.hasResult = false;

    if (res.getMultiTagResult().estimatedPose.isPresent) {
      Transform3d fieldToCamera = res.getMultiTagResult().estimatedPose.best;
      inputs.hasResult = true;
      inputs.estimatedRobotPose =
          new Pose3d()
              .plus(fieldToCamera)
              .relativeTo(VisionBase.m_fieldLayout.getOrigin())
              .plus(kRobotToCamera.inverse());
      inputs.detectedTags =
          res.getMultiTagResult().fiducialIDsUsed.stream().mapToInt(Integer::intValue).toArray();
      inputs.visionMeasurementStdDevs = MULTI_TARGET_STDDEVS;
    } else if (!res.getTargets().isEmpty()) {
      // Find the detected target with the lowest pose ambiguity that is within the field layout
      double lowestAmbiguityScore = Double.POSITIVE_INFINITY;
      PhotonTrackedTarget lowestAmbiguityTarget = null;
      for (var target : res.getTargets()) {
        double targetPoseAmbiguity = target.getPoseAmbiguity();

        if (targetPoseAmbiguity != -1
            && targetPoseAmbiguity < lowestAmbiguityScore
            && VisionBase.m_fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
          lowestAmbiguityScore = targetPoseAmbiguity;
          lowestAmbiguityTarget = target;
        }
      }

      if (lowestAmbiguityTarget != null) {
        var tagPoseOpt = VisionBase.m_fieldLayout.getTagPose(lowestAmbiguityTarget.getFiducialId());
        if (tagPoseOpt.isPresent()) {
          inputs.hasResult = true;
          inputs.estimatedRobotPose =
              tagPoseOpt
                  .get()
                  .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                  .transformBy(kRobotToCamera.inverse());
          inputs.detectedTags =
              res.getTargets().stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
          inputs.visionMeasurementStdDevs = SINGLE_TARGET_STDDEVS;
        }
      }
    }

    if (inputs.hasResult) {
      // Calculate the average distance between the estimated pose and the pose of all detected
      // fiducial targets in three dimensions
      double totalDistance = 0.0;
      // Guaranteed to have at least one target
      int numTags = 0;
      for (int tagId : inputs.detectedTags) {
        var tagPoseOpt = VisionBase.m_fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) continue;
        numTags++;
        var tagPose = tagPoseOpt.get();
        totalDistance +=
            tagPose.getTranslation().getDistance(inputs.estimatedRobotPose.getTranslation());
      }
      double averageDistance = totalDistance / numTags;
      // Increase std devs based on the average distance to the target
      inputs.visionMeasurementStdDevs =
          inputs.visionMeasurementStdDevs.times(1.0 + (Math.pow(averageDistance, 2.0) / numTags));
    } else {
      // Record default values if no results were found
      inputs.estimatedRobotPose = new Pose3d();
      inputs.detectedTags = new int[] {};
      inputs.visionMeasurementStdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0, 0.0, 0.0);
    }
  }
}
