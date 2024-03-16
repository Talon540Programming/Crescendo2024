package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveTeleop;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.oi.ControlsInterface;
import frc.robot.oi.SimKeyboard;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.dynamics.ShooterDynamics;
import frc.robot.subsystems.shooter.dynamics.ShooterState;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseEstimator;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final ControlsInterface controlsInterface = new SimKeyboard();

  private final DriveBase m_drive;
  private final ShooterBase m_shooter;
  private final IntakeBase m_intake;
  private final VisionBase m_vision;

  private final LoggedDashboardChooser<Command> m_autoChooser =
      new LoggedDashboardChooser<>("AutoChooser");

  public RobotContainer() {
    switch (Constants.getRobotMode()) {
      case REAL -> {
        m_drive =
            new DriveBase(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        m_shooter =
            new ShooterBase(
                new ErectorIOSparkMax(), new ShooterModuleIOSparkMax(), new KickupIOSparkMax());
        m_intake =
            new IntakeBase(new WristIOSparkMax(), new RollerIOSparkMax(), new IndexerIOSparkMax());
        m_vision =
            new VisionBase(
                Constants.Vision.configs.stream()
                    .map(
                        v ->
                            new VisionIOPhotonCamera(
                                v.cameraName(), v.robotToCamera(), v.cameraBias()))
                    .toArray(VisionIOPhotonCamera[]::new));
      }
      case SIM -> {
        m_drive =
            new DriveBase(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        m_shooter =
            new ShooterBase(new ErectorIOSim(), new ShooterModuleIOSim(), new KickupIOSim());
        m_intake = new IntakeBase(new WristIOSim(), new RollerIOSim(), new IndexerIOSim());
        m_vision =
            new VisionBase(
                Constants.Vision.configs.stream()
                    .map(
                        v ->
                            new VisionIOSim(
                                v.cameraName(),
                                v.robotToCamera(),
                                v.cameraBias(),
                                v.calibrationPath()))
                    .toArray(VisionIOSim[]::new));
      }
      default -> {
        m_drive =
            new DriveBase(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_shooter =
            new ShooterBase(new ErectorIO() {}, new ShooterModuleIO() {}, new KickupIO() {});
        m_intake = new IntakeBase(new WristIO() {}, new RollerIO() {}, new IndexerIO() {});
        m_vision = new VisionBase(new VisionIO[Constants.Vision.configs.size()]);
      }
    }

    configureAutos();

    if (Constants.TUNING_MODE) {
      configureTunableParameters();
    }

    configureButtonBindings();
  }

  private void configureAutos() {
    m_autoChooser.addDefaultOption("None", Commands.none());
  }

  private void configureTunableParameters() {
    m_autoChooser.addOption(
        "ShooterModuleFFCharacterization", m_shooter.getShooterCharacterizationCommand());
    m_autoChooser.addOption("DriveFFCharacterization", m_drive.getDriveCharacterizationCommand());
  }

  private void configureButtonBindings() {
    m_drive.setDefaultCommand(
        new DriveTeleop(
            m_drive,
            controlsInterface,
            (var pose, var speeds) -> {
              if (ShooterDynamics.inShooterZone(pose)) {
                return Optional.of(
                    ShooterDynamics.calculateRobotSpeakerAngle(pose.getTranslation(), speeds));
              } else if (ShooterDynamics.inIntakeZone(pose)) {
                return Optional.of(
                    ShooterDynamics.calculateRobotIntakeAngle(pose.getTranslation()));
              }

              return Optional.empty();
            }));

    controlsInterface.moduleLock().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    controlsInterface
        .shoot()
        .and(m_shooter::canShoot)
        .onTrue(Commands.runOnce(() -> m_shooter.setKickupVoltage(12.0), m_shooter))
        .onFalse(Commands.runOnce(() -> m_shooter.setKickupVoltage(0.0), m_shooter));

    controlsInterface
        .subwooferPoseOverride()
        .onTrue(
            Commands.runOnce(
                () ->
                    PoseEstimator.getInstance()
                        .resetPose(
                            new Pose2d(
                                AllianceFlipUtil.apply(
                                    FieldConstants.Speaker.centerSpeaker
                                        .getRaw()
                                        .toTranslation2d()
                                        .plus(
                                            new Translation2d(
                                                Constants.ROBOT_LENGTH / 2.0 + 0.75,
                                                new Rotation2d()))),
                                AllianceFlipUtil.apply(new Rotation2d())))));
    controlsInterface
        .sourcePoseOverride()
        .onTrue(
            Commands.runOnce(
                () -> {
                  var wallAngle = FieldConstants.Source.SOURCE_WALL_ANGLE.get();
                  PoseEstimator.getInstance()
                      .resetPose(
                          new Pose2d(
                              AllianceFlipUtil.apply(
                                  FieldConstants.Source.SOURCE_RIGHT_OPENING
                                      .getRaw()
                                      .toTranslation2d()
                                      .plus(
                                          new Translation2d(
                                              Constants.ROBOT_WIDTH / 2.0 + Constants.BUMPER_WIDTH,
                                              AllianceFlipUtil.apply(wallAngle)))),
                              wallAngle));
                }));

    controlsInterface
        .intake()
        .and(() -> !m_shooter.holdingNote())
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                    Commands.runOnce(
                        () -> {
                          m_shooter.setAutoModeEnabled(false);
                          m_shooter.setSetpoint(ShooterState.GROUND_INTAKE_STATE);
                        },
                        m_shooter),
                    Commands.runOnce(
                        () -> m_intake.setWristGoal(Constants.Intake.GROUND_INTAKE_ANGLE),
                        m_intake)),
                Commands.waitUntil(() -> m_shooter.atSetpoint() && m_intake.atWristGoal()),
                Commands.run(
                        () -> {
                          m_intake.setRollersVoltage(12.0);
                          m_intake.setIndexerVoltage(12.0);
                          m_shooter.setKickupVoltage(8.0);
                        },
                        m_intake,
                        m_shooter)
                    .until(m_shooter::holdingNote),
                Commands.waitSeconds(0.25),
                Commands.runOnce(
                    () -> {
                      m_intake.setRollersVoltage(0);
                      m_intake.setIndexerVoltage(0);
                      m_shooter.setKickupVoltage(0);
                      m_intake.setWristGoal(Constants.Intake.STOW_ANGLE);
                      m_shooter.setAutoModeEnabled(true);
                    },
                    m_intake,
                    m_shooter)));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
