//------------------------------------------------------------------------------------
// 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project. 
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;

public final class CatzConstants {

  // ----------------------------------------------------
  //
  //  Robot Modes
  //
  // --------------------------------------------------/
  public static final RobotSenario robotSenario = RobotSenario.TUNING;
  public static final RobotHardwareMode hardwareMode = RobotHardwareMode.REAL;
  private static RobotID robotType = RobotID.SN1;
  private static AlertPriority alertWarningPriority = AlertPriority.ONE;
  public static boolean disableHAL = false;

  public static final double LOOP_TIME = 0.02;

  public static enum RobotSenario {
    TUNING, // In PID enviroment with logged tunable numbers
    PRACTICE, // Driver Practice + Testing
    COMPETITION // Competition Setting
  }

  public static enum RobotHardwareMode {
    REAL,
    SIM,
    REPLAY
  }

  public static RobotID getRobotType() {
    // Checks to ensure that the selected robot Hardware mode is not paired with an illegal Robot Id
    if (RobotBase.isReal() && robotType == RobotID.SN_TEST) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
          .set(true);
      robotType = RobotID.SN2;
    }
    return robotType;
  }

  public static void disableHAL() {
    disableHAL = true;
  }


  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotID.SN_TEST) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robotType != RobotID.SN_TEST || robotSenario == RobotSenario.TUNING) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  public static enum RobotID {
    SN1,
    SN2,
    SN_TEST, // Select alternate test robot parameters
    SN1_2024
  }

  public static enum AllianceColor {
    Blue,
    Red
  }

  public static final class XboxInterfaceConstants {
    // Xbox Driver Ports
    public static final int XBOX_DRV_PORT = 0;
    public static final int XBOX_AUX_PORT = 1;

    // Deadbands
    public static final double kDeadband = 0.1;
  }

  // COLOR CONSTANTS
  public static final class CatzColorConstants {
    public static final Color PHANTOM_SAPPHIRE = new Color(15, 25, 400);
  }

  public static enum AlertPriority {
    ONE, // Includes only critical Error Alerts
    TWO, // Includes critical Error and Warning Alerts
    THREE, // Includes Critical Error, Warning, and Info Alerts
  }
}
