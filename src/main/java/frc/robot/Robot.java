// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggerUtil;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // private Command autonomousCommand;
  private final RobotContainer robotContainer;

  // private double autoStart;
  // private boolean autoMessagePrinted;

  public Robot() {
    robotContainer = new RobotContainer();

    LoggerUtil.initializeLoggerMetadata();

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        var loggerPath = LoggerUtil.getLogPath();
        if (loggerPath.isPresent()) {
          Logger.addDataReceiver(new WPILOGWriter(loggerPath.get().toString()));
        } else {
          DriverStation.reportWarning("Logging USB Drive Not Found. Disabling File Logging", false);
        }

        Logger.addDataReceiver(new RLOGServer());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new RLOGServer());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    // // Run virtual subsystems
    // VirtualSubsystem.periodicAll();

    // Run command scheduler
    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");

    // Handle Alerts
    // Robot container periodic methods
    robotContainer.updateAlerts();
    robotContainer.updateDashboardOutputs();

    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  // /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
  // */
  // @Override
  // public void autonomousInit() {}

  // /** This function is called periodically during autonomous. */
  // @Override
  // public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
