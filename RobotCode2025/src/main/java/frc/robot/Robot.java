// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.RobotHardwareMode;
import frc.robot.CatzConstants.RobotID;
import frc.robot.CatzConstants.RobotSenario;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.LEDs.CatzLED;
import frc.robot.Commands.ControllerModeAbstraction;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.VirtualSubsystem;
import frc.robot.Utilities.Alert.AlertType;
import lombok.Getter;


public class Robot extends LoggedRobot {
  //-------------------------------------------------------------------------------------------------------------
  //  Essential Robot.java object declaration
  //--------------------------------------------------------------------------------------------------------------
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Optional<Alliance> alliance = Optional.empty();

  //-------------------------------------------------------------------------------------------------------------
  //  MISC
  //--------------------------------------------------------------------------------------------------------------
  // Robot Timers
  private final Timer DISABLED_TIMER           = new Timer(); // TODO make these finals to conventions
  private final Timer CAN_INITIAL_ERROR_TIMER  = new Timer();
  private final Timer CAN_ERROR_TIMER          = new Timer();
  private final Timer CANIVORE_ERROR_TIMER     = new Timer();

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
  @Getter private static double autoElapsedTime = 0.0;


  //--------------------------------------------------------------------------------------------------------
  //        Alerts
  //--------------------------------------------------------------------------------------------------------
  // CAN
  private final Alert canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);

  // Battery Alerts
  private final Alert lowBatteryAlert = new Alert("Battery voltage is very low, consider turning off the robot or replacing the battery.", AlertType.WARNING);
  private final Alert sameBatteryAlert = new Alert("The battery has not been changed since the last match.", AlertType.WARNING);

  // Garbage Collection Alerts
  private final Alert GC_COLLECTION_ALERT = new Alert("Please wait to enable, collecting garbage. 🗑️", AlertType.WARNING); //TODO reconfigure on time 
  private int garbageCollectionCounter = 0;

  // DriverStation related alerts
  private final Alert DS_DISCONNECT_ALERT = new Alert("Driverstation is not online, alliance selection will not work", AlertType.ERROR);
  private final Alert FMS_DISCONNECT_ALERT = new Alert("fms is offline, robot cannot compete in match", AlertType.ERROR);

  // Last deployment logging
  Date date = Calendar.getInstance().getTime();	
  SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd kk.mm.ss");;
  String dateFormatted = sdf.format(date);
  private final Alert LAST_DEPLOYMENT_WARNING = new Alert("Last Deployment: " + dateFormatted , AlertType.INFO);

  // reset Position Logging
  public static boolean isResetPositionUsedInAuto = false;


  //--------------------------------------------------------------------------------------------------------
  //
  //        Robot
  //
  //--------------------------------------------------------------------------------------------------------
  @Override
  public void robotInit() {
    System.gc();

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
        case 0:
            Logger.recordMetadata("GitDirty", "All changes committed");
            break;
        case 1:
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
        default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
    }

    // Set up data receivers & replay source
    switch (CatzConstants.hardwareMode) {
        case REAL:
            // Running on a real robot, log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new WPILOGWriter("E:/Logs"));
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case SIM:
            // Running a physics simulator, log to NT
            //Logger.addDataReceiver(new WPILOGWriter("F:/robotics code projects/loggingfiles/"));
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case REPLAY:
            // Replaying a log, set up replay source
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            break;
    }

    // Start AdvantageKit logger //TODO enable this in autonomous and telop init for comp setting // make advantage kit an on demand feature
    Logger.start();

    // Instantiate robotContainer
    m_robotContainer = new RobotContainer();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });

    // Reset alert timers
    CAN_INITIAL_ERROR_TIMER.restart();
    CAN_ERROR_TIMER.restart();
    CANIVORE_ERROR_TIMER.restart();
    DISABLED_TIMER.restart();

    RobotController.setBrownoutVoltage(6.0);

    // Print out Catz Constant enums
    System.out.println("Enviroment: " + CatzConstants.robotSenario.toString());
    System.out.println("Mode: " + CatzConstants.hardwareMode.toString());
    System.out.println("Type: " + CatzConstants.getRobotType().toString());


    // Run hardware mode check
    if(Robot.isReal()) { //REAL ROBOT
      if(CatzConstants.hardwareMode == RobotHardwareMode.SIM) {
        System.out.println("Wrong Robot Constant selection, Check CatzConstants hardwareMode");
        System.exit(0);
      }

      if(CatzConstants.getRobotType() == RobotID.SN_TEST) {
        System.out.println("Wrong Robot ID selection, Check CatzConstants robotID");
        System.exit(0);
      }

    } else { //SIM ROBOT
      if(CatzConstants.hardwareMode == RobotHardwareMode.REAL) {
        System.out.println("Wrong Robot Constant selection, Check CatzConstants hardwareMode");
        System.exit(0);
      }

      if(CatzConstants.getRobotType() != RobotID.SN_TEST) {
        if(CatzConstants.hardwareMode == RobotHardwareMode.SIM) {
          System.out.println("Wrong Robot ID selection, Check CatzConstants robotID"); 
          System.exit(0);
        }
      }
    }

    DriverStation.silenceJoystickConnectionWarning(true);
  }



  @Override
  public void robotPeriodic() {
    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();

    // Print auto duration
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
        CatzLED.getInstance().autoFinished = true;
        CatzLED.getInstance().autoFinishedTime = Timer.getFPGATimestamp();
      }
    }

    // Robot container periodic methods
    m_robotContainer.checkControllers();
    m_robotContainer.updateDashboardOutputs();
    m_robotContainer.updateAlerts();

    // Check CAN status
    var canStatus = RobotController.getCANStatus(); // TODO find benifits of getCANStatus and evaluate whether it  
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      CAN_ERROR_TIMER.restart();
    }
    canErrorAlert.set(
        !CAN_ERROR_TIMER.hasElapsed(CAN_ERROR_TIME_THRESHOLD)
            && !CAN_INITIAL_ERROR_TIMER.hasElapsed(CAN_ERROR_TIME_THRESHOLD));

    // Low battery alert
    if (DriverStation.isEnabled()) {
      DISABLED_TIMER.reset();
    }
    if (RobotController.getBatteryVoltage() <= LOW_BATTERY_VOLTAGE
        && DISABLED_TIMER.hasElapsed(LOW_BATTERY_DISABLED_TIME)) {
      lowBatteryAlert.set(true);
      CatzLED.getInstance().lowBatteryAlert = true;
    }
  }
  


  //--------------------------------------------------------------------------------------------------------
  //
  //        Disabled
  //
  //--------------------------------------------------------------------------------------------------------
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
    if((garbageCollectionCounter > 5*60*4)) { // 1 second * 60sec * 4 min
      System.gc();
      garbageCollectionCounter = 0;
    }
    garbageCollectionCounter++;

  }



  @Override
  public void disabledExit() {}


  //--------------------------------------------------------------------------------------------------------
  //
  //        Autonomous
  //
  //--------------------------------------------------------------------------------------------------------  
  @Override
  public void autonomousInit() {
    // deployment benchmark
    LAST_DEPLOYMENT_WARNING.set(true);
    autoStart = Timer.getFPGATimestamp();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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


  //--------------------------------------------------------------------------------------------------------
  //
  //        teleop
  //
  //--------------------------------------------------------------------------------------------------------
  @Override
  public void teleopInit() {
    // deployment benchmark
    LAST_DEPLOYMENT_WARNING.set(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    teleStart = Timer.getFPGATimestamp();
  }



  @Override
  public void teleopPeriodic() {
    teleElapsedTime = Timer.getFPGATimestamp() - teleStart;

    ControllerModeAbstraction.periodicDebug();
  }



  @Override
  public void teleopExit() {}


  //--------------------------------------------------------------------------------------------------------
  //
  //        test
  //
  //--------------------------------------------------------------------------------------------------------
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}