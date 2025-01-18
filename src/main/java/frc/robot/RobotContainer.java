package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommandFactory;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.*;

public class RobotContainer {
  private final DriveBase m_drive;

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    System.out.println("CHECKPOINT 1");
    switch (Constants.getRobotMode()) {
      case REAL -> {
        m_drive =
            new DriveBase(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
      }
      case SIM -> {
        m_drive =
            new DriveBase(
                new GyroIO() {},
                new ModuleIOSim() {},
                new ModuleIOSim() {},
                new ModuleIOSim() {},
                new ModuleIOSim() {});
      }
      default -> {
        m_drive =
            new DriveBase(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }
    }

    configureAutos();

    if (Constants.TUNING_MODE) {
      configureTunableParameters();
    }

    configureButtonBindings();
  }

  private void configureAutos() {
  }

  private void configureTunableParameters() {
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
    return Commands.none();
  }
}
