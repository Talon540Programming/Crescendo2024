package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ShooterTeleop;
import frc.robot.constants.Constants;
import frc.robot.oi.ControlsInterface;
import frc.robot.oi.SrimanXbox;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.dynamics.ShooterDynamics;
import frc.robot.subsystems.vision.*;
import frc.robot.util.PoseEstimator;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final ControlsInterface controlsInterface = new SrimanXbox();

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
                    .map(v -> new VisionIOPhotonCamera(v.cameraName(), v.robotToCamera()))
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
                    .map(v -> new VisionIOSim(v.cameraName(), v.robotToCamera()))
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
        "DriveDynamicForward", m_drive.characterizeDriveDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "DriveDynamicReverse", m_drive.characterizeDriveDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "DriveQuasistaticForward",
        m_drive.characterizeDriveQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "DriveQuasistaticReverse",
        m_drive.characterizeDriveQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "ErectorDynamicForward",
        m_shooter.characterizeErectorDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "ErectorDynamicReverse",
        m_shooter.characterizeErectorDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "ErectorQuasistaticForward",
        m_shooter.characterizeErectorQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "ErectorQuasistaticReverse",
        m_shooter.characterizeErectorQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "ShooterDynamicForward",
        m_shooter.characterizeShooterDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "ShooterDynamicReverse",
        m_shooter.characterizeShooterDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "ShooterQuasistaticForward",
        m_shooter.characterizeShooterQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "ShooterQuasistaticReverse",
        m_shooter.characterizeShooterQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "WristDynamicForward", m_intake.characterizeWristDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "WristDynamicReverse", m_intake.characterizeWristDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "WristQuasistaticForward",
        m_intake.characterizeWristQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "WristQuasistaticReverse",
        m_intake.characterizeWristQuasistatic(SysIdRoutine.Direction.kReverse));
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

    // Drive teleop override button
    controlsInterface
        .trajectoryOverride()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_drive.getCurrentCommand().cancel();
                  m_drive.stop();
                }));

    m_shooter.setDefaultCommand(
        new ShooterTeleop(
            m_shooter,
            () -> PoseEstimator.getInstance().getPose(),
            m_drive::getVelocity,
            controlsInterface));

    // Shoot and intake override
    controlsInterface
        .shootPoseOverride()
        .onTrue(
            Commands.runOnce(
                () -> {
                  PoseEstimator.getInstance().resetPose(Constants.Poses.SUBWF_SHOOT_POSE);
                }));

    controlsInterface
        .feederIntakePoseOverride()
        .onTrue(
            Commands.runOnce(
                () -> {
                  PoseEstimator.getInstance().resetPose(Constants.Poses.FEEDER_INTAKE_POSE);
                }));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
