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

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  private static RobotType robotType = RobotType.OUTREACHBOT;
  // Allows tunable values to be changed when enabled. Also adds tunable selectors to AutoSelector
  public static final boolean TUNING_MODE = true;
  // Disable the AdvantageKit logger from running
  // public static final boolean ENABLE_LOGGING = false;
  // Disable LEDs, will reduce software and electrical overhead but disable hardware alerts
  // public static final boolean ENABLE_LEDs = false;

  public static final double kLoopPeriodSecs = 0.02;

  // Available Robot Modes
  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  // Returns Current Robot Mode
  public static Mode getMode() {
    return switch (robotType) {
      case OUTREACHBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  // Available Robot Types
  public enum RobotType {
    SIMBOT,
    OUTREACHBOT
  }

  // Returns the current robot type
  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert(
              "Invalid robot selected, using competition robot as default.", Alert.AlertType.kError)
          .set(true);
      robotType = RobotType.OUTREACHBOT;
    }
    return robotType;
  }
}
