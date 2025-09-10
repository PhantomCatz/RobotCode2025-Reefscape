package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;

import choreo.auto.AutoFactory;

public final class CatzConstants {

  // ----------------------------------------------------
  //
  //  Robot Modes
  //
  // --------------------------------------------------/
  public static final RobotScenario robotScenario = RobotScenario.TUNING;
  public static final RobotHardwareMode hardwareMode = RobotHardwareMode.SIM;
  private static RobotID robotType = RobotID.SN_TEST;
  private static AlertPriority alertWarningPriority = AlertPriority.ONE;
  public static boolean disableHAL = false;

  public static final double LOOP_TIME = 0.02;

  public static final AutoFactory autoFactory = new AutoFactory(
                                                  CatzRobotTracker.Instance::getEstimatedPose,
                                                  CatzRobotTracker.Instance::resetPose,
                                                  CatzDrivetrain.Instance::followChoreoTrajectoryExecute,
                                                  true,
                                                  CatzDrivetrain.Instance
                                                );

  public static enum RobotScenario {
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
      if (robotType != RobotID.SN_TEST || robotScenario == RobotScenario.TUNING) {
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
