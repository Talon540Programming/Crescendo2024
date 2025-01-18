package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.Constants;

public class ModuleIOSim implements ModuleIO {
    private final DCMotor m_driveMotorModel;
    private final DCMotor m_turnMotorModel;

    private final DCMotorSim m_driveSim;
    private final DCMotorSim m_turnSim;

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        this.m_driveMotorModel = DCMotor.getNEO(1);
        this.m_turnMotorModel = DCMotor.getNEO(1);
        
        this.m_driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(this.m_driveMotorModel, 0.025, DriveBase.kDriveGearing),
            this.m_driveMotorModel);

        this.m_turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(this.m_turnMotorModel, 0.004, DriveBase.kTurnGearing),
            this.m_turnMotorModel);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        m_driveSim.update(Constants.kLoopPeriodSecs);
        m_turnSim.update(Constants.kLoopPeriodSecs);

        inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = new double[] {Math.abs(m_driveSim.getCurrentDrawAmps())};

        inputs.turnAbsolutePosition = new Rotation2d(m_turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
        inputs.turnPosition = new Rotation2d(m_turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = new double[] {Math.abs(m_turnSim.getCurrentDrawAmps())};


        // System.out.println("drivePosition" + inputs.drivePositionRad);
        // System.out.println("turnPosition" + inputs.turnPosition);

    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        m_driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        m_turnSim.setInputVoltage(turnAppliedVolts);
    }
    
}
