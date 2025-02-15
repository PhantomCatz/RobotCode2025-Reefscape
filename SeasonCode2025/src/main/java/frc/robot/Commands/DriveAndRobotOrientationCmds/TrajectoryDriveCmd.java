// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Utilities.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
  public static final double ALLOWABLE_ROTATION_ERROR = 2.0;
  public static final double ALLOWABLE_VEL_ERROR = 0.2;
  private static final double TIMEOUT_SCALAR = 5;
  private static final double CONVERGE_DISTANCE = 1.0;

  // Subsystems
  private CatzDrivetrain m_driveTrain;
  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();

  // Trajectory variables
  private HolonomicDriveController hocontroller;
  private PathFollowingController ppHoController;
  private PathPlannerTrajectory trajectory;
  private Trajectory testTrajectory;
  private PathPlannerPath path;
  private boolean atTarget = false;
  private double pathTimeOut = -999.0;
  private Timer timer = new Timer();
  private boolean autoalign;

  // Event Command variables
  private final EventScheduler eventScheduler;
  private final Map<Command, Boolean> currentEventCommands = new HashMap();
  private final List<Pair<Double, Command>> untriggeredEvents = new ArrayList();
  private boolean isEventCommandRunning = false;

  private double translationError = Double.MAX_VALUE;

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

    if(Robot.isFirstPath && DriverStation.isAutonomous()){
      try {
        tracker.resetPose(usePath.getStartingHolonomicPose().get());
        Robot.isFirstPath = false;
      } catch (NoSuchElementException e) {
        e.printStackTrace();
      }
    }


    ChassisSpeeds currentSpeeds =
        DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(tracker.getCurrentModuleStates());
    // If we provide an initial speed of zero the trajectory will take an infinite time to finish
    // (divide by 0) and not be sampleable
    if (Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vxMetersPerSecond) < 1e-6) {
      currentSpeeds = DriveConstants.NON_ZERO_CHASSIS_SPEED;
    }
    // Construct trajectory
    this.trajectory =
        new PathPlannerTrajectory(
            usePath,
            currentSpeeds,
            tracker.getEstimatedPose().getRotation(),
            DriveConstants.TRAJECTORY_CONFIG);

    hocontroller = DriveConstants.getNewHolController();
    ppHoController = DriveConstants.getNewPathFollowingController();
    pathTimeOut = trajectory.getTotalTimeSeconds() * TIMEOUT_SCALAR;

    // Reset
    PathPlannerLogging.logActivePath(usePath);
    PPLibTelemetry.setCurrentPath(usePath);

    eventScheduler.initialize(trajectory);
    this.timer.reset();
    this.timer.start();
  } // end of initialize()

  // ---------------------------------------------------------------------------------------------
  //
  // Execute
  //
  // ---------------------------------------------------------------------------------------------
  @Override
  public void execute() {
    double currentTime = this.timer.get();
    // Getters from pathplanner and current robot pose
    PathPlannerTrajectoryState goal =
        trajectory.sample(Math.min(currentTime, trajectory.getTotalTimeSeconds()));
    Pose2d currentPose = tracker.getEstimatedPose();
    ChassisSpeeds currentSpeeds =
        DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(m_driveTrain.getModuleStates());
    double currentVel =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    // Trajectory Executor

    // -------------------------------------------------------------------------------------
    // Convert PP trajectory into a wpilib trajectory type
    // Only takes in the current robot position
    // Does not take acceleration to be used with the internal WPILIB trajectory library
    // Holonomic drive controller only relies on its current position, not its velocity because the
    // target velocity is used as a ff
    // -------------------------------------------------------------------------------------
    Trajectory.State state =
        new Trajectory.State(
            currentTime,
            goal.linearVelocity * DriveConstants.TRAJECTORY_FF_SCALAR,
            0.0, // TODO verify if this does what we want it to do
            new Pose2d(goal.pose.getTranslation(), goal.heading),
            0.0);

    // construct chassisspeeds
    ChassisSpeeds adjustedSpeeds =
        hocontroller.calculate(currentPose, state, goal.pose.getRotation());

    if(autoalign){
      // Graph x^2/(1+x^2) on desmos
      // Smaller speed for closer distances
      double x = Math.pow(CONVERGE_DISTANCE * translationError, 2);
      adjustedSpeeds = adjustedSpeeds.times(x / (x+1));
    }
    System.out.println(adjustedSpeeds);

    // send to drivetrain
    m_driveTrain.drive(adjustedSpeeds);
    // Log desired pose
    Logger.recordOutput("CatzRobotTracker/Desired Auto Pose", goal.pose);

    // ---------------------------------------------------------------------------------------------------------------------------
    // Named Commands
    // ---------------------------------------------------------------------------------------------------------------------------
    eventScheduler.execute(currentTime);

    // ---------------------------------------------------------------------------------------------------------------------------
    //  Logging
    // ---------------------------------------------------------------------------------------------------------------------------
    PPLibTelemetry.setCurrentPose(currentPose);
    PathPlannerLogging.logCurrentPose(currentPose);

    PPLibTelemetry.setTargetPose(goal.pose);
    PathPlannerLogging.logTargetPose(goal.pose);

    PPLibTelemetry.setVelocities(
        currentVel,
        goal.linearVelocity,
        currentSpeeds.omegaRadiansPerSecond,
        goal.heading.getRadians());
    debugLogsTrajectory();
  } // end of execute

  // ---------------------------------------------------------------------------------------------
  //
  // Debug Logs
  //
  // ---------------------------------------------------------------------------------------------
  public void debugLogsTrajectory() {
    // Logger.recordOutput("Desired Auto Pose", new Pose2d(state.poseMeters.getTranslation(),
    // goal.targetHolonomicRotation));
    // Logger.recordOutput("Adjusted Speeds X", adjustedSpeeds.vxMetersPerSecond);
    // Logger.recordOutput("Adjusted Speeds Y", adjustedSpeeds.vyMetersPerSecond);
    // Logger.recordOutput("Trajectory Goal MPS", state.velocityMetersPerSecond);
    // Logger.recordOutput("PathPlanner Goal MPS", goal.velocityMps);

    // System.out.println(goal.getTargetHolonomicPose());
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      System.out.println("trajectory following was interrupted");
    }

    timer.stop(); // Stop timer
    if(interrupted)
    {
      System.out.println("OH NO I GOT INTERUPTED HOW RUDE");
    }
    {
      System.out.println("trajectory done");
      m_driveTrain.stopDriving();
    }

    System.out.println("trajectory done");

    timer.stop();
    PathPlannerAuto.currentPathName = "";
    PathPlannerAuto.setCurrentTrajectory(null);
    PathPlannerLogging.logActivePath(null);

    eventScheduler.end();
  }

  @Override
  public boolean isFinished() {
    // Finish command if the robot is near goal (and if robot velocity is zero if goal velocity is zero)
    PathPlannerTrajectoryState endState = trajectory.getEndState();

    double currentPosX = tracker.getEstimatedPose().getX();
    double currentPosY = tracker.getEstimatedPose().getY();
    double currentRotation = tracker.getEstimatedPose().getRotation().getDegrees();

    double desiredPosX = endState.pose.getX();
    double desiredPosY = endState.pose.getY();
    double desiredRotation = endState.pose.getRotation().getDegrees();

    double desiredMPS = trajectory.getEndState().linearVelocity;
    ChassisSpeeds currentChassisSpeeds = tracker.getRobotChassisSpeeds();
    double currentMPS =
        Math.hypot(
            Math.hypot(
                currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond),
            currentChassisSpeeds.omegaRadiansPerSecond);

    double xError = Math.abs(desiredPosX - currentPosX);
    double yError = Math.abs(desiredPosY - currentPosY);
    double rotationError = Math.abs(desiredRotation - currentRotation);
    if (rotationError > 180) {
      rotationError = 360 - rotationError;
    }
    translationError = Math.hypot(xError, yError);

    // Command not intended to end
    if (autoalign){
      return false;
    }

    // Finish command if the total time the path takes is over
    if (timer.hasElapsed(pathTimeOut) && !isEventCommandRunning){
      return true;
    }

    return
      xError < ALLOWABLE_POSE_ERROR &&
      yError < ALLOWABLE_POSE_ERROR &&
      rotationError < ALLOWABLE_ROTATION_ERROR &&
      (desiredMPS == 0 || currentMPS < ALLOWABLE_VEL_ERROR);
  }
}
