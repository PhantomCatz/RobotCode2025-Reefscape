package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.AutoRoutineSelector;
import frc.robot.CatzConstants.RobotHardwareMode;
import frc.robot.CatzConstants.RobotID;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.CatzAlgaePivot;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.CatzAlgaePivot.AlgaePivotPosition;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.CatzVision;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.ControllerLEDState;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;
import frc.robot.Utilities.MotorUtil.NeutralMode;
import lombok.Getter;
import frc.robot.Utilities.VirtualSubsystem;
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
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;




public class Robot extends LoggedRobot {
  // -------------------------------------------------------------------------------------------------------------
  //  Essential Robot.java object declaration
  // --------------------------------------------------------------------------------------------------------------
  private Command m_autonomousCommand;
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
  public static double autoStart = 0.0;
  private boolean autoMessagePrinted;
  @Getter
  private static double teleElapsedTime = 0.0;
  @Getter
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

  private final RobotContainer container = RobotContainer.Instance;
  private final CatzLED LED = CatzLED.Instance;

  // --------------------------------------------------------------------------------------------------------
  //
  //        Robot
  //
  // --------------------------------------------------------------------------------------------------------
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
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new RLOGServer());
        Logger.addDataReceiver(new WPILOGWriter("/Logs"));

        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        // Logger.addDataReceiver(new WPILOGWriter("F:/robotics code projects/loggingfiles/"));
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

    // Start AdvantageKit logger //TODO enable this in autonomous and telop init for comp setting //
    // make advantage kit an on demand feature
    Logger.start();

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

    // Set Brownout Voltatage to WPILIB recommendations
    RobotController.setBrownoutVoltage(6.3);

    // Print out Catz Constant enums
    System.out.println("Enviroment: " + CatzConstants.robotScenario.toString());
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

    SmartDashboard.putData("Auton Selector", AutoRoutineSelector.Instance.getAutoChooser());
    // CommandScheduler.getInstance().setPeriod(CatzConstants.LOOP_TIME); //TODO should we add this?
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
        LED.autoFinishedTime = Timer.getFPGATimestamp();
      }
    }

    //-------------------------------------------------------------------------------------------------
    // Robot container periodic methods
    //-------------------------------------------------------------------------------------------------
    container.checkControllers();
    container.updateDashboardOutputs();
    container.updateAlerts();

    //-------------------------------------------------------------------------------------------------
    // Check CAN status
    // For Transimit Errors, recieve errors, busOffCounts, percent util, txfullcount
    //-------------------------------------------------------------------------------------------------
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      CAN_ERROR_TIMER.restart();
    }
    CAN_ERROR_ALERT.set(!CAN_ERROR_TIMER.hasElapsed(CAN_ERROR_TIME_THRESHOLD) && !CAN_INITIAL_ERROR_TIMER.hasElapsed(CAN_ERROR_TIME_THRESHOLD));
    Logger.recordOutput("CANErrors/Transmit Errors", canStatus.transmitErrorCount);
    Logger.recordOutput("CANErrors/Recieve Errors count", canStatus.receiveErrorCount);
    Logger.recordOutput("CANErrors/Bus Off Count", canStatus.busOffCount);
    Logger.recordOutput("CANErrors/Percent Utilization", canStatus.percentBusUtilization);
    Logger.recordOutput("CANErrors/TxFullCount", canStatus.txFullCount);

    //------------------------------------------------------------------------------------------------
    // Low battery alert to determine whether necessary to put in new battery
    //------------------------------------------------------------------------------------------------
    if (DriverStation.isEnabled()) {
      DISABLED_TIMER.reset();
    }

    if (RobotController.getBatteryVoltage() <= LOW_BATTERY_VOLTAGE
        && DISABLED_TIMER.hasElapsed(LOW_BATTERY_DISABLED_TIME)) {
      LOW_BATTERY_ALERT.set(true);
      LED.setControllerState(ControllerLEDState.lowBatteryAlert);
    }

  }

  // --------------------------------------------------------------------------------------------------------
  //
  //        Disabled
  //
  // --------------------------------------------------------------------------------------------------------
  @Override
  public void disabledInit() {
    CatzRampPivot.Instance.setNeutralMode(NeutralMode.COAST);
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

    // Checked leds
    if(CatzVision.Instance.getTagId(1) == 263 || CatzVision.Instance.getTagId(0) == 263) {
      LED.setControllerState(ControllerLEDState.ledChecked);
    }
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
    CatzRampPivot.Instance.setNeutralMode(NeutralMode.BRAKE);
    CatzAlgaePivot.Instance.setAlgaePivotPos(AlgaePivotPosition.STOW);


    autoStart = Timer.getFPGATimestamp();
    m_autonomousCommand = CatzSuperstructure.Instance.scoreLevelTwoAutomated();
    //m_autonomousCommand = AutoRoutineSelector.Instance.getSelectedCommand();
    CatzRampPivot.Instance.Ramp_Intake_Pos().withTimeout(1.0);
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
    } else {
      CatzRampPivot.Instance.Ramp_Intake_Pos().withTimeout(1.0);
    }



    teleStart = Timer.getFPGATimestamp();
    CatzRampPivot.Instance.setNeutralMode(NeutralMode.BRAKE);
    CatzAlgaePivot.Instance.setAlgaePivotPos(AlgaePivotPosition.STOW);
    //CatzRobotTracker.Instance.resetPose(m_robotContainer.getAutonomous().calculateReefPos(2, LeftRight.LEFT));
    //For limelight error calculations CatzRobotTracker.Instance.resetPose(FlippingUtil.flipFieldPose(new Pose2d(Reef.center.minus(new Translation2d(Units.inchesToMeters(32.305+14.5), 0)), Rotation2d.kZero)));
    //CatzRobotTracker.Instance.resetPose(new Pose2d());
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
    CatzRobotTracker.Instance.resetPose(CatzVision.Instance.getPoseObservation()[0].pose().toPose2d());
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
