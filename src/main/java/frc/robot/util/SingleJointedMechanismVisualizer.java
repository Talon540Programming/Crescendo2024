package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class SingleJointedMechanismVisualizer implements AutoCloseable {
  private final String m_mechanismName;
  private final String m_logKey;

  private final Mechanism2d m_mechanism;
  private final MechanismLigament2d m_mechanismLigament;

  private final Pose3d m_pivotPose;

  public SingleJointedMechanismVisualizer(
      String mechanismName,
      String logKey,
      double ligamentLengthMeters,
      Pose3d pivotPose,
      Rotation2d defaultAngle,
      double mechanismCanvasWidthMeters,
      double mechanismCanvasHeightMeters) {
    m_mechanismName = mechanismName;
    m_logKey = logKey;

    m_pivotPose = pivotPose;

    m_mechanism =
        new Mechanism2d(
            mechanismCanvasWidthMeters, mechanismCanvasHeightMeters, new Color8Bit(14, 17, 23));
    MechanismRoot2d pivot = m_mechanism.getRoot("Mechanism Fulcrum", 1, pivotPose.getZ());

    m_mechanismLigament =
        pivot.append(
            new MechanismLigament2d(
                "jointLig",
                ligamentLengthMeters,
                defaultAngle.getDegrees(),
                4,
                new Color8Bit(Color.kBlue)));
  }

  public void update(Rotation2d mechanismAngle) {
    m_mechanismLigament.setAngle(mechanismAngle.getDegrees());
    Logger.recordOutput(m_mechanismName + "/Mechanism2d/" + m_logKey, m_mechanism);

    Logger.recordOutput(
        m_mechanismName + "/Mechanism3d/" + m_logKey,
        new Pose3d(
            m_pivotPose.getTranslation(), new Rotation3d(0, mechanismAngle.getRadians(), 0)));
  }

  @Override
  public void close() {
    m_mechanismLigament.close();
    m_mechanism.close();
  }
}
