package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;

public class IntakeDynamics {
    public record IntakeState(
        Rotation2d angle,
        double rollerPercent
    ) {
        public static IntakeState STARTING_STATE = new IntakeState(Constants.Intake.kMaxIntakeAngle, 0);
        public static IntakeState TRAVEL_STATE = new IntakeState(Rotation2d.fromDegrees(69), 0);

        @Override
        public boolean equals(Object obj) {
            if (obj instanceof IntakeState other) {
                return angle.equals(other.angle)
                    && Math.abs(rollerPercent - other.rollerPercent) <= 1e-2;
            }
            return false;
        }
    }
}
