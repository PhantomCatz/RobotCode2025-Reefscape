package frc.robot;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants.DRIVE_CURRENT_LIMIT;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants.ROBOT_MASS;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants.ROBOT_MOI;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants.TRAJECTORY_MODULE_TRANSLATIONS;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants.TREAD_COEF_FRICTION;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;



/***
 * CatzConstants
 * 
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 *         This class is where reusable constants are defined
 ***/
public final class CatzConstants {

  //----------------------------------------------------
  //
  //  Robot Modes
  //
  //--------------------------------------------------/
  public static final RobotSenario robotSenario      = RobotSenario.PRACTICE;
  public static final RobotHardwareMode hardwareMode = RobotHardwareMode.REAL;
  private static RobotID robotType                   = RobotID.SN_TEST;
  private static AlertPriority alertWarningPriority  = AlertPriority.ONE;
  
  public static final double LOOP_TIME = 0.02;


  public static enum RobotSenario {
    TUNING, //In PID enviroment with logged tunable numbers
    PRACTICE, //Driver Practice + Testing
    COMPETITION //Competition Setting
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

  public static enum RobotID {
    SN1,
    SN2,
    SN_TEST, // Select alternate test robot parameters
    SN1_2024
  }

  public static enum AllianceColor {
    Blue, Red
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
    TWO,  // Includes critical Error and Warning Alerts
    THREE, // Includes Critical Error, Warning, and Info Alerts
  }

}