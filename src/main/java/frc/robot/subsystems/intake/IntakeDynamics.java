package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;

public class IntakeDynamics {
    public record IntakeState(
        Rotation2d angle,
        double intakeVelocityMetersPerSec
    ) {
        public static IntakeState STARTING_STATE = new IntakeState(Constants.Intake.kMaxIntakeAngle, 0);
        public static IntakeState TRAVEL_STATE = new IntakeState(Rotation2d.fromDegrees(69), 0);

        @Override
        public boolean equals(Object obj) {
            if (obj instanceof IntakeState) {
                return angle.equals(((IntakeState) obj).angle)
                    && Math.abs(intakeVelocityMetersPerSec - ((IntakeState) obj).intakeVelocityMetersPerSec)
                        <= 1e-5;
            }
            return false;
        }
    }
}
