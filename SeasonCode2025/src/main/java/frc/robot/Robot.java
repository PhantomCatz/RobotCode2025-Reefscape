// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants.RobotHardwareMode;
import frc.robot.CatzConstants.RobotID;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;
import frc.robot.Utilities.VirtualSubsystem;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

public class Robot extends TimedRobot {
  // -------------------------------------------------------------------------------------------------------------
  //  Essential Robot.java object declaration
  // --------------------------------------------------------------------------------------------------------------
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Optional<Alliance> alliance = Optional.empty();

  // -------------------------------------------------------------------------------------------------------------
  //  MISC
  // --------------------------------------------------------------------------------------------------------------
  // Robot Timers
  private final Timer DISABLED_TIMER = new Timer(); // TODO make these finals to conventions
  private final Timer CAN_INITIAL_ERROR_TIMER = new Timer();
  private final Timer CAN_ERROR_TIMER = new Timer();
  private final Timer CANIVORE_ERROR_TIMER = new Timer();

  // Can Error Detection variables
  private static final double CAN_ERROR_TIME_THRESHOLD = 0.5; // Seconds to disable alert //TODO
  private static final double CANIVORE_ERROR_TIME_THRESHOLD = 0.5;
  private static final double LOW_BATTERY_VOLTAGE = 11.8;
  private static final double LOW_BATTERY_DISABLED_TIME = 1.5;

  // Timer related variables
  private double teleStart;
  private double autoStart;
  private boolean autoMessagePrinted;
  private static double teleElapsedTime = 0.0;
  private static double autoElapsedTime = 0.0;

  // --------------------------------------------------------------------------------------------------------
  //        Alerts
  // --------------------------------------------------------------------------------------------------------
  // CAN
  private final Alert CAN_ERROR_ALERT = new Alert("CAN errors detected", AlertType.kError);

  // Battery Alerts
  private final Alert LOW_BATTERY_ALERT  = new Alert("Battery voltage is below 11 volts, replace battery", AlertType.kWarning);
  private final Alert SAME_BATTERY_ALERT = new Alert("The battery not changed since last match.", AlertType.kError);

  // Garbage Collection Alerts
  private final Alert GC_COLLECTION_ALERT = new Alert("Please wait to enable, collecting garbage.", AlertType.kWarning);
  private int garbageCollectionCounter = 0;

  // DriverStation related alerts
  private final Alert DS_DISCONNECT_ALERT = new Alert("Driverstation is not online, alliance selection will not work", AlertType.kError);
  private final Alert FMS_DISCONNECT_ALERT = new Alert("fms is offline, robot cannot compete in match", AlertType.kError);

  // Last deployment logging
  Date date = Calendar.getInstance().getTime();
  SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd kk.mm.ss");
  String dateFormatted = sdf.format(date);
  private final Alert LAST_DEPLOYMENT_WARNING = new Alert("Last Deployment: " + dateFormatted, AlertType.kInfo);

  // reset Position Logging
  public static boolean isResetPositionUsedInAuto = false;

  // --------------------------------------------------------------------------------------------------------
  //
  //        Robot
  //
  // --------------------------------------------------------------------------------------------------------
  @Override
  public void robotInit() {

    // Instantiate robotContainer
    m_robotContainer = new RobotContainer();

    // Reset alert timers
    CAN_INITIAL_ERROR_TIMER.restart();
    CAN_ERROR_TIMER.restart();
    CANIVORE_ERROR_TIMER.restart();
    DISABLED_TIMER.restart();

    // Set Brownout Voltatage to WPILIB recommendations
    RobotController.setBrownoutVoltage(6.3);

    // Print out Catz Constant enums
    System.out.println("Enviroment: " + CatzConstants.robotSenario.toString());
    System.out.println("Mode: " + CatzConstants.hardwareMode.toString());
    System.out.println("Type: " + CatzConstants.getRobotType().toString());

    // Run hardware mode check
    if (Robot.isReal()) { // REAL ROBOT
      if (CatzConstants.hardwareMode == RobotHardwareMode.SIM) {
        System.out.println("Wrong Robot Constant selection, Check CatzConstants hardwareMode");
        System.exit(0);
      }

      if (CatzConstants.getRobotType() == RobotID.SN_TEST) {
        System.out.println("Wrong Robot ID selection, Check CatzConstants robotID");
        System.exit(0);
      }

    } else { // SIM ROBOT
      if (CatzConstants.hardwareMode == RobotHardwareMode.REAL) {
        System.out.println("Wrong Robot Constant selection, Check CatzConstants hardwareMode");
        System.exit(0);
      }

      if (CatzConstants.getRobotType() != RobotID.SN_TEST) {
        if (CatzConstants.hardwareMode == RobotHardwareMode.SIM) {
          System.out.println("Wrong Robot ID selection, Check CatzConstants robotID");
          System.exit(0);
        }
      }
    }

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    //------------------------------------------------------------------------------------------------
    // Run Subsystem Periodics
    //------------------------------------------------------------------------------------------------
    VirtualSubsystem.periodicAll();

    CommandScheduler.getInstance().run();


    //-------------------------------------------------------------------------------------------------
    // Print auto duration
    //-------------------------------------------------------------------------------------------------
    if (m_autonomousCommand != null) {
      if (!m_autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    //-------------------------------------------------------------------------------------------------
    // Robot container periodic methods
    //-------------------------------------------------------------------------------------------------
    m_robotContainer.checkControllers();
    m_robotContainer.updateDashboardOutputs();
    m_robotContainer.updateAlerts();

    //-------------------------------------------------------------------------------------------------
    // Check CAN status
    // For Transimit Errors, recieve errors, busOffCounts, percent util, txfullcount
    //-------------------------------------------------------------------------------------------------
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      CAN_ERROR_TIMER.restart();
    }
    CAN_ERROR_ALERT.set(!CAN_ERROR_TIMER.hasElapsed(CAN_ERROR_TIME_THRESHOLD) && !CAN_INITIAL_ERROR_TIMER.hasElapsed(CAN_ERROR_TIME_THRESHOLD));

    //------------------------------------------------------------------------------------------------
    // Low battery alert to determine whether necessary to put in new battery
    //------------------------------------------------------------------------------------------------
    if (DriverStation.isEnabled()) {
      DISABLED_TIMER.reset();
    }

    if (RobotController.getBatteryVoltage() <= LOW_BATTERY_VOLTAGE
        && DISABLED_TIMER.hasElapsed(LOW_BATTERY_DISABLED_TIME)) {
      LOW_BATTERY_ALERT.set(true);
    }
  }

  // --------------------------------------------------------------------------------------------------------
  //
  //        Disabled
  //
  // --------------------------------------------------------------------------------------------------------
  @Override
  public void disabledInit() {
    isResetPositionUsedInAuto = false;
  }

  @Override
  public void disabledPeriodic() {
    // Driver Station Alerts
    DS_DISCONNECT_ALERT.set(!DriverStation.isDSAttached());
    FMS_DISCONNECT_ALERT.set(!DriverStation.isFMSAttached());

    // deployment benchmark
    LAST_DEPLOYMENT_WARNING.set(false);

    // Garbage Collection alert
    GC_COLLECTION_ALERT.set(Timer.getFPGATimestamp() < 45.0);
    if ((garbageCollectionCounter > 5 * 60 * 4)) { // 1 second * 60sec * 4 min
      System.gc();
      GC_COLLECTION_ALERT.set(false);
      GC_COLLECTION_ALERT.set(true);
      garbageCollectionCounter = 0;
    }
    garbageCollectionCounter++;
  }

  @Override
  public void disabledExit() {}

  // --------------------------------------------------------------------------------------------------------
  //
  //        Autonomous
  //
  // --------------------------------------------------------------------------------------------------------
  public static boolean isFirstPath = true;

  @Override
  public void autonomousInit() {
    isFirstPath = true;
    // deployment benchmark
    LAST_DEPLOYMENT_WARNING.set(true);
    autoStart = Timer.getFPGATimestamp();
    m_autonomousCommand = new SequentialCommandGroup();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    autoElapsedTime = Timer.getFPGATimestamp() - autoStart;
  }

  @Override
  public void autonomousExit() {}

  // --------------------------------------------------------------------------------------------------------
  //
  //        teleop
  //
  // --------------------------------------------------------------------------------------------------------
  @Override
  public void teleopInit() {
    // deployment benchmark
    LAST_DEPLOYMENT_WARNING.set(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

    }

    teleStart = Timer.getFPGATimestamp();
    //CatzRobotTracker.getInstance().resetPose(m_robotContainer.getAutonomous().calculateReefPos(2, LeftRight.LEFT));

  //  CatzRobotTracker.getInstance().resetPose(new Pose2d(Reef.center.minus(new Translation2d(Units.inchesToMeters(52.743), 0)), Rotation2d.k180deg));

  }

  @Override
  public void teleopPeriodic() {

    teleElapsedTime = Timer.getFPGATimestamp() - teleStart;

  }

  @Override
  public void teleopExit() {}

  // --------------------------------------------------------------------------------------------------------
  //
  //        test
  //
  // --------------------------------------------------------------------------------------------------------
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
