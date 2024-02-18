package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.HardwareIds;
import frc.robot.util.ThroughBoreEncoder;

public class WristIOSparkMax implements WristIO {
    private final CANSparkMax m_motor;
    private final ThroughBoreEncoder m_encoder;

    public WristIOSparkMax() {
        m_motor =
            new CANSparkMax(HardwareIds.COMP_2024.kIntakeWristId, CANSparkMax.MotorType.kBrushless);
        m_encoder = 
            new ThroughBoreEncoder(
                HardwareIds.COMP_2024.kIntakeWristAbsoluteEncoderPort,
                HardwareIds.COMP_2024.kIntakeWristRelativeEncoderAPort,
                HardwareIds.COMP_2024.kIntakeWristRelativeEncoderBPort,
                Rotation2d.fromRadians(3.1415926535 - (Math.PI / 2.0)));
        
        m_motor.restoreFactoryDefaults();
        m_motor.setCANTimeout(250);
        m_motor.setSmartCurrentLimit(40);
        m_motor.enableVoltageCompensation(12);
        m_motor.setCANTimeout(0);
        m_motor.burnFlash();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.velocityRadPerSec = m_encoder.getVelocityRadPerSecond();
        inputs.absoluteAngle = m_encoder.getAbsolutePosition();
        inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = new double[] {m_motor.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        m_motor.setIdleMode(enable ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}
