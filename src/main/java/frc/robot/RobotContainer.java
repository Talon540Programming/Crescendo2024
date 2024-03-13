package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.DriveCommandFactory;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final DriveBase m_drive;
  private final ShooterBase m_shooter;
  private final IntakeBase m_intake;
  private final VisionBase m_vision;

  private final CommandXboxController controller = new CommandXboxController(0);

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
        DriveCommandFactory.joystickDrive(
            m_drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            0.1));
    controller.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
