package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommandFactory;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareIds;
import frc.robot.subsystems.drive.*;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PathPlannerUtil;
import frc.robot.util.PoseEstimator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final DriveBase m_drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.getRobotMode()) {
      case REAL -> {
        m_drive =
            new DriveBase(
                new GyroIOPigeon2(HardwareIds.kPigeonId),
                new ModuleIOSparkMax(
                    HardwareIds.kFrontLeftDriveId,
                    HardwareIds.kFrontLeftTurnId,
                    HardwareIds.kFrontLeftEncoderId),
                new ModuleIOSparkMax(
                    HardwareIds.kFrontRightDriveId,
                    HardwareIds.kFrontRightTurnId,
                    HardwareIds.kFrontRightEncoderId),
                new ModuleIOSparkMax(
                    HardwareIds.kBackLeftDriveId,
                    HardwareIds.kBackLeftTurnId,
                    HardwareIds.kBackLeftEncoderId),
                new ModuleIOSparkMax(
                    HardwareIds.kBackRightDriveId,
                    HardwareIds.kBackRightTurnId,
                    HardwareIds.kBackRightEncoderId));
      }
      case SIM -> {
        m_drive =
            new DriveBase(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
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

    // Configure PathPlanner
    AutoBuilder.configureHolonomic(
        () -> PoseEstimator.getInstance().getPose(),
        (pose) -> PoseEstimator.getInstance().resetPose(pose),
        () -> Constants.Drivetrain.m_kinematics.toChassisSpeeds(m_drive.getModuleStates()),
        m_drive::runVelocity,
        new HolonomicPathFollowerConfig(
            Constants.Drivetrain.kMaxLinearVelocityMetersPerSecond,
            Constants.Drivetrain.kDriveBaseRadiusMeters,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        m_drive);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) ->
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Choices",
            PathPlannerUtil.configureChooserWithPaths(AutoBuilder.buildAutoChooser()));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_drive.setDefaultCommand(
        DriveCommandFactory.sprintJoystickDrive(
            m_drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            controller.getHID()::getRightBumper,
            0.5,
            0.1));
    controller.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
