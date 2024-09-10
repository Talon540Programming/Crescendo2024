package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveTeleop;
import frc.robot.constants.Constants;
import frc.robot.oi.ControlsInterface;
import frc.robot.oi.SingleXbox;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.dynamics.ShooterState;

public class RobotContainer {
  private final ControlsInterface controlsInterface = new SingleXbox();

  private final DriveBase m_drive;
  private final ShooterBase m_shooter;

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
                new ErectorIO() {}, new ShooterModuleIOSparkMax(), new KickupIOSparkMax());
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
      }
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_drive.setDefaultCommand(new DriveTeleop(m_drive, controlsInterface));

    controlsInterface.moduleLock().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    controlsInterface
        .shoot()
        .and(m_shooter::holdingNote)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> m_shooter.setSetpoint(new ShooterState(Rotation2d.fromDegrees(45), 3.5))),
                Commands.waitUntil(m_shooter::atVelocitySetpoint),
                Commands.runOnce(() -> m_shooter.setKickupVoltage(12.0), m_shooter)))
        .onFalse(
            Commands.sequence(
                Commands.runOnce(() -> m_shooter.setKickupVoltage(0.0), m_shooter),
                Commands.runOnce(
                    () ->
                        m_shooter.setSetpoint(new ShooterState(Rotation2d.fromDegrees(45), 0.0)))));

    // controlsInterface
    //     .subwooferPoseOverride()
    //     .onTrue(
    //         Commands.runOnce(
    //             () ->
    //                 PoseEstimator.getInstance()
    //                     .resetPose(
    //                         new Pose2d(
    //                             AllianceFlipUtil.apply(
    //                                 FieldConstants.Speaker.centerSpeaker
    //                                     .getRaw()
    //                                     .toTranslation2d()
    //                                     .plus(
    //                                         new Translation2d(
    //                                             Constants.ROBOT_LENGTH / 2.0 + 0.75,
    //                                             new Rotation2d()))),
    //                             AllianceFlipUtil.apply(new Rotation2d())))));
    // controlsInterface
    //     .sourcePoseOverride()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               var wallAngle = FieldConstants.Source.SOURCE_WALL_ANGLE.get();
    //               PoseEstimator.getInstance()
    //                   .resetPose(
    //                       new Pose2d(
    //                           AllianceFlipUtil.apply(
    //                               FieldConstants.Source.SOURCE_RIGHT_OPENING
    //                                   .getRaw()
    //                                   .toTranslation2d()
    //                                   .plus(
    //                                       new Translation2d(
    //                                           Constants.ROBOT_WIDTH / 2.0 +
    // Constants.BUMPER_WIDTH,
    //                                           AllianceFlipUtil.apply(wallAngle)))),
    //                           wallAngle));
    //             }));

    // controlsInterface
    //     .intake()
    //     .and(() -> !m_shooter.holdingNote())
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.runOnce(
    //                 () -> {
    //                   m_shooter.setAutoModeEnabled(false);
    //                   m_shooter.setSetpoint(ShooterState.GROUND_INTAKE_STATE);
    //                   m_intake.setWristGoal(Constants.Intake.GROUND_INTAKE_ANGLE);
    //                 },
    //                 m_shooter,
    //                 m_intake),
    //             Commands.waitUntil(() -> m_shooter.atSetpoint() && m_intake.atWristGoal()),
    //             Commands.run(
    //                     () -> {
    //                       m_intake.setRollersVoltage(8.0);
    //                       m_intake.setIndexerVoltage(8.0);
    //                       m_shooter.setKickupVoltage(8.0);
    //                     },
    //                     m_intake,
    //                     m_shooter)
    //                 .until(m_shooter::holdingNote)
    //                 .withTimeout(7.5),
    //             Commands.waitSeconds(0.25),
    //             Commands.runOnce(
    //                 () -> {
    //                   m_intake.setRollersVoltage(0);
    //                   m_intake.setIndexerVoltage(0);
    //                   m_shooter.setKickupVoltage(0);
    //                   m_intake.setWristGoal(Constants.Intake.STOW_ANGLE);
    //                   m_shooter.setAutoModeEnabled(true);
    //                 },
    //                 m_intake,
    //                 m_shooter)));

    // controlsInterface
    //     .ejectIndexer()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.runOnce(
    //                 () -> m_intake.setWristGoal(Constants.Intake.STOW_ANGLE), m_intake),
    //             Commands.waitUntil(m_intake::atWristGoal),
    //             Commands.runOnce(() -> m_intake.setIndexerVoltage(-8.0), m_intake),
    //             Commands.waitSeconds(1.5),
    //             Commands.runOnce(() -> m_intake.setIndexerVoltage(0), m_intake)));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
