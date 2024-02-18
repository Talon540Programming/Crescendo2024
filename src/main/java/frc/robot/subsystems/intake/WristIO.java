package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO {
    @AutoLog
    class WristIOInputs {
        public double velocityRadPerSec = 0.0;
        public Rotation2d absoluteAngle = new Rotation2d();
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

    public default void updateInputs(WristIOInputs inputs) {};

    public default void setVoltage(double voltage) {};

    public default void setBrakeMode(boolean enable) {};
}
