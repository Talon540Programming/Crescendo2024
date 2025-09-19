package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
// import frc.robot.commands.AutoBuilder;
// import frc.robot.subsystems.shooter.ShooterBase;
// import frc.robot.subsystems.shooter.ShooterIO;
// import frc.robot.subsystems.shooter.ShooterIOSim;
// import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.drive.*;
import frc.robot.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({DoublePressTracker.class, TriggerUtil.class})
public class RobotContainer {
  /* Subsystems */
  private DriveBase driveBase;
  // private ShooterBase intakeBase;

  /* Controller */
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Alert controllerDisconnected =
      new Alert("Main controller disconnected (port 0).", Alert.AlertType.kWarning);

  /* Dashboard inputs */
  // private final LoggedDashboardChooser<Command> autoChooser;

  // private final LoggedNetworkNumber endgameAlert1 =
  //     new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  // private final LoggedNetworkNumber endgameAlert2 =
  //     new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        driveBase =
            new DriveBase(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        // shooterBase = new ShooterBase(new ShooterIOSpark());
      }
      case SIM -> {
        driveBase =
            new DriveBase(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // shooterBase = new ShooterBase(new ShooterIOSim());
      }
    }

    /* Initialize no-op implementations */
    if (driveBase == null) {
      driveBase =
          new DriveBase(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    // if (shooterBase == null) {
    //     shooterBase = new ShooterBase(new ShooterIO() {});
    // }

    /* Set up auto routines */
    // var autoBuilder = new AutoBuilder(driveBase, shooterBase);
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // autoChooser.addDefaultOption("Noting", Commands.none());
    // autoChooser.addOption("Taxi", autoBuilder.taxi());
    // autoChooser.addOption("SingleCoralCenterStart", autoBuilder.centerStartSingle());
    // autoChooser.addOption("Multi", autoBuilder.sideStartMulti(false));
    // autoChooser.addOption("DeadreckonedMulti", autoBuilder.sideStartMulti(true));

    // if (Constants.TUNING_MODE) {
    //   // Set up Characterization routines
    //   autoChooser.addOption(
    //       "Drive Wheel Radius Characterization", driveBase.wheelRadiusCharacterization());
    //   autoChooser.addOption(
    //       "Drive Simple FF Characterization", driveBase.feedforwardCharacterization());
    // }

    // LoggedDashboardChooser<Boolean> mirror =
    //     new LoggedDashboardChooser<>("Starting on Processor Side?");
    // mirror.addDefaultOption("Yes", false);
    // mirror.addOption("No", true);
    // MirrorUtil.setMirror(mirror::get);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Drive suppliers. Allows both driver and operator to have control over bot.
    DoubleSupplier driverX = () -> -controller.getLeftY();
    DoubleSupplier driverY = () -> -controller.getLeftX();
    DoubleSupplier driverOmega = () -> -controller.getRightX();
    // TODO
    // BooleanSupplier robotRelative =
    //     () -> controller.leftBumper().and(controller.rightBumper()).getAsBoolean();
    BooleanSupplier robotRelative = () -> false;

    // Joystick drive command (driver and operator)
    Supplier<Command> joystickDriveCommandFactory =
        () -> DriveCommands.joystickDrive(driveBase, driverX, driverY, driverOmega, robotRelative);
    driveBase.setDefaultCommand(joystickDriveCommandFactory.get());

    /* Assign buttons here */
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    controllerDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));
  }

  // public Command getAutonomousCommand() {
  //   return autoChooser.get();
  // }
}
