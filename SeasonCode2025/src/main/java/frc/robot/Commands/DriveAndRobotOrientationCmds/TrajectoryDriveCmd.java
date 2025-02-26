// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Utilities.AllianceFlipUtil;
import java.util.Collections;
import java.util.NoSuchElementException;
import java.util.Set;


import org.littletonrobotics.junction.Logger;

/**************************************************************************************************
 *
 *
 *
 * TrajectoryDriveCmd
 *
 *
 *
 **************************************************************************************************/

public class TrajectoryDriveCmd extends Command {
  // Trajectory constants
  public static final double ALLOWABLE_POSE_ERROR = 0.05;
  public static final double ALLOWABLE_AUTOAIM_ERROR = 0.05;
  public static final double ALLOWABLE_ROTATION_ERROR = 2.0;
  public static final double ALLOWABLE_POSE_ERROR_PID = 0.02;
  public static final double ALLOWABLE_ROTATION_ERROR_PID = 0.5;
  public static final double ALLOWABLE_VEL_ERROR = 0.2;
  public static final double ALLOWABLE_OMEGA_ERROR = 3.0;
  private static final double TIMEOUT_SCALAR = 5.0;
  private static final double CONVERGE_DISTANCE = 1.0;
  private static final double DIVERGE_TIME = 1.0;
  private final double ALLOWABLE_VISION_ADJUST = 9e-4; //TODO tune

  // Subsystems
  private CatzDrivetrain m_driveTrain;
  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();

  // Trajectory variables
  private HolonomicDriveController hocontroller;
  private PathPlannerTrajectory trajectory;
  private PathPlannerPath path;
  private double pathTimeOut = -999.0;
  private Timer timer = new Timer();
  private boolean autoalign = false;
  private double translationError = FieldConstants.FIELD_LENGTH_MTRS * 2;

  // PID aim variables
  private boolean isPIDAimEnabled = false;
  private PIDController poseAdjustController = new PIDController(3.0, 0.0, 0.0);
  private PIDController thethaController  = new PIDController(0.4, 0.0, 0.0);
  private ChassisSpeeds PIDaimSpeeds = new ChassisSpeeds();
  private Pose2d goalPIDAimPose           = new Pose2d();

  // Swerve Drive Variables
  private ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();


  // Event Command variables
  private final EventScheduler eventScheduler;
  private boolean isEventCommandRunning = false;

  // ---------------------------------------------------------------------------------------------
  //
  // Trajectory Drive Command Constructor
  //
  // ---------------------------------------------------------------------------------------------
  public TrajectoryDriveCmd(PathPlannerPath newPath, CatzDrivetrain drivetrain, boolean autoalign) {
    this.path = newPath;
    this.m_driveTrain = drivetrain;
    this.autoalign = autoalign;
    this.eventScheduler = new EventScheduler();
    addRequirements(m_driveTrain);

    // Add all event scheduler requirements to this command's requirements
    var eventReqs = EventScheduler.getSchedulerRequirements(this.path);
    if (!Collections.disjoint(Set.of(m_driveTrain), eventReqs)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventReqs);

    thethaController.enableContinuousInput(-180.0, 180.0);
  }

  // ---------------------------------------------------------------------------------------------
  //
  // Initialize
  //
  // ---------------------------------------------------------------------------------------------
  @Override
  public void initialize() {
    // Flip path if necessary
    System.out.println("trajec start");
    PathPlannerPath usePath = path;
    if (AllianceFlipUtil.shouldFlipToRed()) {
      usePath = path.flipPath();
    }

    // Pose Reseting
    if(Robot.isFirstPath && DriverStation.isAutonomous()){
      try {
        tracker.resetPose(usePath.getStartingHolonomicPose().get());
        Robot.isFirstPath = false;
      } catch (NoSuchElementException e) {
        e.printStackTrace();
      }
    }

    // Collect current drive state
    ChassisSpeeds currentSpeeds = DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(tracker.getCurrentModuleStates());

    // If we provide an initial speed of zero the trajectory will take an infinite time to finish
    // (divide by 0) and not be sampleable
    if (Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vxMetersPerSecond) < 1e-6) {
      currentSpeeds = DriveConstants.NON_ZERO_CHASSIS_SPEED;
    }

    //Initialize variables
    isPIDAimEnabled = false;
    goalPIDAimPose  = new Pose2d(
                            usePath.getPathPoses().get(usePath.getPathPoses().size() - 1).getX(),
                            usePath.getPathPoses().get(usePath.getPathPoses().size() - 1).getY(),
                            usePath.getGoalEndState().rotation()
    );

    // Determine Trajectory vs Simple PID path following
    if(usePath.getAllPathPoints().size() <= 2) {
      System.out.println(isPIDAimEnabled);
      // isPIDAimEnabled = true; //TODO is this needed?
      // System.out.println("PID AIM!!!!!!!!!!!!!!!");
    } else {
      // Construct trajectory
      this.trajectory =
          new PathPlannerTrajectory(
              usePath,
              currentSpeeds,
              tracker.getEstimatedPose().getRotation(),
              DriveConstants.TRAJECTORY_CONFIG);

      hocontroller = DriveConstants.getNewHolController();
      pathTimeOut = trajectory.getTotalTimeSeconds() * TIMEOUT_SCALAR;

      eventScheduler.initialize(trajectory);
    }

    // System.out.println("current " + tracker.getEstimatedPose());
    // System.out.println("start " + this.trajectory.getInitialPose());
    // System.out.println("end " + this.trajectory.getEndState().pose);

    // Reset
    PathPlannerLogging.logActivePath(usePath);
    PPLibTelemetry.setCurrentPath(usePath);

    this.timer.reset();
    this.timer.start();

    if(trajectory == null) {
      isPIDAimEnabled = true;
    }
  } // end of initialize()

  // ---------------------------------------------------------------------------------------------
  //
  // Execute
  //
  // ---------------------------------------------------------------------------------------------
  @Override
  public void execute() {
    // Collect instananous variables
    double currentTime = this.timer.get();
    Pose2d currentPose              = tracker.getEstimatedPose();
    ChassisSpeeds currentSpeeds     = DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(m_driveTrain.getModuleStates());
    double currentVel               = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);


    // Simple PID aim
    if(isPIDAimEnabled) {
      double pidSpeedX = poseAdjustController.calculate(currentPose.getX(), goalPIDAimPose.getX());
      double pidSpeedY = poseAdjustController.calculate(currentPose.getY(), goalPIDAimPose.getY());
      double thethaSpeed = thethaController.calculate(currentPose.getRotation().getDegrees(), goalPIDAimPose.getRotation().getDegrees());
      System.out.println(thethaSpeed);
      PIDaimSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(pidSpeedX, pidSpeedY, thethaSpeed, tracker.getEstimatedPose().getRotation());
      adjustedSpeeds = PIDaimSpeeds;

    // Trajectory Aim
    } else {
      // -------------------------------------------------------------------------------------
      // Convert PP trajectory into a wpilib trajectory type
      // Only takes in the current robot position
      // Does not take acceleration to be used with the internal WPILIB trajectory library
      // Holonomic drive controller only relies on its current position, not its velocity because the
      // target velocity is used as a ff
      // -------------------------------------------------------------------------------------
      PathPlannerTrajectoryState goal = trajectory.sample(Math.min(currentTime, trajectory.getTotalTimeSeconds()));
      Trajectory.State state =
          new Trajectory.State(
              currentTime,
              goal.linearVelocity * DriveConstants.TRAJECTORY_FF_SCALAR,
              0.0,
              new Pose2d(goal.pose.getTranslation(), goal.heading),
              0.0);

      // construct chassisspeeds
      adjustedSpeeds = hocontroller.calculate(currentPose, state, goal.pose.getRotation());

      // Cusps x/(1+x) Lower speed for closer distances to prevent jittering //TODO tune
      if(currentTime <= 1.5){
        adjustedSpeeds = adjustedSpeeds.times(DIVERGE_TIME * currentTime / (DIVERGE_TIME * currentTime + 1));
      }
      if(autoalign && translationError < 1.0){
        adjustedSpeeds = adjustedSpeeds.times(CONVERGE_DISTANCE * translationError / (CONVERGE_DISTANCE * translationError + 1));
      }
      if(Double.isNaN(adjustedSpeeds.vxMetersPerSecond) || Double.isNaN(adjustedSpeeds.vyMetersPerSecond) || Double.isNaN(adjustedSpeeds.omegaRadiansPerSecond)){
        adjustedSpeeds = new ChassisSpeeds();
      }

      // Logging
      Logger.recordOutput("CatzRobotTracker/Desired Auto Pose", goal.pose);

      PPLibTelemetry.setCurrentPose(currentPose);
      PathPlannerLogging.logCurrentPose(currentPose);

      PPLibTelemetry.setTargetPose(goal.pose);
      PathPlannerLogging.logTargetPose(goal.pose);

      PPLibTelemetry.setVelocities(
          currentVel,
          goal.linearVelocity,
          currentSpeeds.omegaRadiansPerSecond,
          goal.heading.getRadians()
      );
    }


    // send to drivetrain
    m_driveTrain.drive(adjustedSpeeds);


    // Named Commands
    eventScheduler.execute(currentTime);


    //  Logging
    debugLogsTrajectory();
  } // end of execute

  // ---------------------------------------------------------------------------------------------
  //
  // Debug Logs
  //
  // ---------------------------------------------------------------------------------------------
  public void debugLogsTrajectory() {
    Logger.recordOutput("Trajectory/PID aim pose", goalPIDAimPose);
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      System.out.println("OH NO I GOT INTERUPTED HOW RUDE");
    }
    System.out.println("trajectory done");

    timer.stop(); // Stop timer
    m_driveTrain.stopDriving();

    PathPlannerAuto.currentPathName = "";
    PathPlannerAuto.setCurrentTrajectory(null);
    PathPlannerLogging.logActivePath(null);

    eventScheduler.end();
  }

  @Override
  public boolean isFinished(){
    // Command not intended to end for con
    // if (autoalign){
    //   return false;
    // }
    // System.out.println("vision: " +tracker.getDEstimatedPose().getTranslation().getNorm() );
    System.out.println("vision: " + (tracker.getDEstimatedPose().getTranslation().getNorm()) + " pose: " + translationError);



    if (timer.hasElapsed(pathTimeOut) && !isEventCommandRunning){
      System.out.println("timed out!!@)!*()*!)(#*)");
      return true;
    }

    if (autoalign && tracker.getDEstimatedPose().getTranslation().getNorm() > ALLOWABLE_VISION_ADJUST){
      System.out.println("vision is not true!@!@!)@(!)@()(!@)");
      return false;
    }
    // Finish command if the total time the path takes is over


    if(autoalign){
      System.out.println("passed vision check!!!@)*)(*)(@*#()*@)#(*)");
      return isWithinThreshold(ALLOWABLE_AUTOAIM_ERROR);
    }else{
      System.out.println("not auto align!@!@!");
      return isWithinThreshold(ALLOWABLE_POSE_ERROR);

    }
  }

  public boolean isWithinThreshold(double poseError) {
    // Check if the robot is near goal (and if robot velocity is zero if goal velocity is zero)
    PathPlannerTrajectoryState endState = trajectory.getEndState();

    double currentPosX = tracker.getEstimatedPose().getX();
    double currentPosY = tracker.getEstimatedPose().getY();
    double currentRotation = tracker.getEstimatedPose().getRotation().getDegrees();

    double desiredPosX = endState.pose.getX();
    double desiredPosY = endState.pose.getY();
    double desiredRotation = endState.pose.getRotation().getDegrees();

    ChassisSpeeds currentChassisSpeeds = tracker.getRobotChassisSpeeds();
    double desiredMPS = trajectory.getEndState().linearVelocity;
    double currentMPS = Math.hypot(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);
    double currentRPS = currentChassisSpeeds.omegaRadiansPerSecond;

    double xError = Math.abs(desiredPosX - currentPosX);
    double yError = Math.abs(desiredPosY - currentPosY);
    double rotationError = Math.abs(desiredRotation - currentRotation);
    if (rotationError > 180) {
      rotationError = 360 - rotationError;
    }
    translationError = Math.hypot(xError, yError);


    System.out.println("rotait: " + (rotationError < ALLOWABLE_OMEGA_ERROR));
    System.out.println("speed: " + (desiredMPS == 0.0 || (currentMPS < ALLOWABLE_VEL_ERROR && currentRPS < ALLOWABLE_OMEGA_ERROR)));

    if(isPIDAimEnabled) {
      return
        xError < ALLOWABLE_POSE_ERROR_PID &&
        yError < ALLOWABLE_POSE_ERROR_PID &&
        rotationError < ALLOWABLE_ROTATION_ERROR &&
        (desiredMPS == 0 || (currentMPS < ALLOWABLE_VEL_ERROR && currentRPS < ALLOWABLE_OMEGA_ERROR));
    } else {
      return
        xError < poseError &&
        yError < poseError &&
        rotationError < ALLOWABLE_OMEGA_ERROR &&
        (desiredMPS == 0.0 || (currentMPS < ALLOWABLE_VEL_ERROR && currentRPS < ALLOWABLE_OMEGA_ERROR));
    }
  }
}
