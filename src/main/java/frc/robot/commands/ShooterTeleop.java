package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.shooter.ShooterBase;
import frc.robot.subsystems.shooter.dynamics.ShooterDynamics;
import frc.robot.subsystems.shooter.dynamics.ShooterState;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterTeleop extends Command {
  private static final double SHOOTING_LOW_VELOCITY = 12.5;

  private final ShooterBase shooterBase;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final BooleanSupplier ejectSupplier;

  public ShooterTeleop(
      ShooterBase shooterBase,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BooleanSupplier ejectSupplier) {
    addRequirements(shooterBase);

    this.shooterBase = shooterBase;

    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.ejectSupplier = ejectSupplier;
  }

  public ShooterTeleop(
      ShooterBase shooterBase,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      OperatorOI oi) {
    this(shooterBase, poseSupplier, speedsSupplier, () -> oi.shoot().getAsBoolean());
  }

  @Override
  public void execute() {
    var currentPose = poseSupplier.get();
    var currentSpeeds = speedsSupplier.get();
    boolean holdingNote = shooterBase.holdingNote();

    if (ShooterDynamics.inShooterZone(currentPose) && holdingNote) {
      boolean eject = ejectSupplier.getAsBoolean();
      ShooterDynamics.calculateSpeakerState(currentPose, currentSpeeds)
          .ifPresent(
              v -> {
                // Not shooting yet, bring to mid-level velocity. This will reduce the convergence
                // rate for when we do shoot.
                shooterBase.setSetpoint(
                    eject ? v : new ShooterState(v.angle, SHOOTING_LOW_VELOCITY));

                boolean withinTolerance =
                    ShooterDynamics.withinSpeakerYawTolerance(currentPose, currentSpeeds);
                Logger.recordOutput("ShooterTeleop/Eject", eject);
                Logger.recordOutput("ShooterTeleop/WithinYawTolerance", withinTolerance);

                // Ensure we are at the shooter setpoint and yaw is aligned before ejecting the note
                if (eject && shooterBase.atSetpoint() && withinTolerance) {
                  shooterBase.setKickupVoltage(12.0);
                } else {
                  shooterBase.setKickupVoltage(0.0);
                }
              });
    } else if (ShooterDynamics.inIntakeZone(currentPose)) {
      ShooterDynamics.calculateIntakeState(currentPose)
          .ifPresent(
              v -> {
                // Don't bring the shooter down until we leave the intake zone to prevent damage
                shooterBase.setSetpoint(!holdingNote ? v : ShooterState.STARTING_STATE);
                shooterBase.setKickupVoltage(!holdingNote ? -8.0 : 0);
              });
    } else {
      shooterBase.setSetpoint(ShooterState.TRAVEL_STATE);
      shooterBase.setKickupVoltage(0.0);
    }
  }
}
