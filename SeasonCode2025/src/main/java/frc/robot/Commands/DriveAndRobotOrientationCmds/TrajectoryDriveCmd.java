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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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
  public static final double ALLOWABLE_AUTOAIM_ERROR = 0.02;
  public static final double ALLOWABLE_ROTATION_ERROR = 1.0;
  public static final double ALLOWABLE_VEL_ERROR = 0.2;
  public static final double ALLOWABLE_OMEGA_ERROR = 3.0;
  private static final double TIMEOUT_SCALAR = 50.0;
  private static final double CONVERGE_DISTANCE = 0.05;
  private final double ALLOWABLE_VISION_ADJUST = 4e-3; //TODO tune

  // Subsystems
  private CatzDrivetrain m_driveTrain;
  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
  private final RobotContainer container;

  // Trajectory variables
  private HolonomicDriveController hocontroller;
  private PathPlannerTrajectory trajectory;
  private PathPlannerPath path;
  private double pathTimeOut = -999.0;
  private Timer timer = new Timer();
  private boolean autoalign = false;
  private double translationError = FieldConstants.FIELD_LENGTH_MTRS * 2;

  // Swerve Drive Variables
  private ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();


  // Event Command variables
  private final EventScheduler eventScheduler;
  private boolean isEventCommandRunning = false;

  private final XboxController xboxDrv;

  private boolean isBugged = false;

  // ---------------------------------------------------------------------------------------------
  //
  // Trajectory Drive Command Constructor
  //
  // ---------------------------------------------------------------------------------------------
  public TrajectoryDriveCmd(PathPlannerPath newPath, CatzDrivetrain drivetrain, boolean autoalign, RobotContainer container) {
    this.path = newPath;
    this.m_driveTrain = drivetrain;
    this.autoalign = autoalign;
    this.container = container;
    this.eventScheduler = new EventScheduler();
    xboxDrv = null;
    addRequirements(m_driveTrain);

    // Add all event scheduler requirements to this command's requirements
    var eventReqs = EventScheduler.getSchedulerRequirements(this.path);
    if (!Collections.disjoint(Set.of(m_driveTrain), eventReqs)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventReqs);
  }

  //For NBA. Used to rumble the controller when done driving
  public TrajectoryDriveCmd(PathPlannerPath newPath, CatzDrivetrain drivetrain, boolean autoalign, RobotContainer container, XboxController xboxDrv) {
    this.path = newPath;
    this.m_driveTrain = drivetrain;
    this.autoalign = autoalign;
    this.container = container;
    this.eventScheduler = new EventScheduler();
    this.xboxDrv = xboxDrv;
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
    try{
      // Flip path if necessary
      System.out.println("trajec start");
      PathPlannerPath usePath = path;
      if (AllianceFlipUtil.shouldFlipToRed()) {
        usePath = path.flipPath();
        System.out.println("Path flipped!!!!!");
      }


      // Pose Reseting
      if (Robot.isFirstPath && DriverStation.isAutonomous()) {
        try {
          tracker.resetPose(usePath.getStartingHolonomicPose().get());
          Robot.isFirstPath = false;
        } catch (NoSuchElementException e) {
          e.printStackTrace();
        }
      }

      // Collect current drive state
      ChassisSpeeds currentSpeeds = DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(tracker.getCurrentModuleStates());

      // Construct trajectory
      if(autoalign){
        this.trajectory = new PathPlannerTrajectory(
          usePath,
          currentSpeeds, //TODO make it not zero if its a thing thingy y esdpoifi
          tracker.getEstimatedPose().getRotation(),
          DriveConstants.TRAJ_ROBOT_CONFIG
        );
      }else{
        this.trajectory = new PathPlannerTrajectory(
          usePath,

          currentSpeeds, //TODO make it not zero if its a thing thingy y esdpoifi
          tracker.getEstimatedPose().getRotation(),
          DriveConstants.TRAJ_ROBOT_CONFIG
        );
      }


      hocontroller = DriveConstants.getNewHolController();
      pathTimeOut = trajectory.getTotalTimeSeconds() * TIMEOUT_SCALAR;

      // Event marker initialize
      eventScheduler.initialize(trajectory);

      // System.out.println("current " + tracker.getEstimatedPose());
      // System.out.println("start " + this.trajectory.getInitialPose());
      // System.out.println("end " + this.trajectory.getEndState().pose);

      // Reset
      PathPlannerLogging.logActivePath(usePath);
      PPLibTelemetry.setCurrentPath(usePath);

      this.timer.reset();
      this.timer.start();
    }catch(Exception e){
      isBugged = true;
      e.printStackTrace();
    }
    //
  } // end of initialize()



  // ---------------------------------------------------------------------------------------------
  //
  // Execute
  //
  // ---------------------------------------------------------------------------------------------
  @Override
  public void execute() {
    if(this.trajectory == null || isBugged) return;

    // Collect instananous variables
    double currentTime = timer.get();
    Pose2d currentPose = tracker.getEstimatedPose();
    ChassisSpeeds currentSpeeds = DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(m_driveTrain.getModuleStates());
    double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);


    // -------------------------------------------------------------------------------------
    // Convert PP trajectory into a wpilib trajectory type
    // Only takes in the current robot position
    // Does not take acceleration to be used with the internal WPILIB trajectory
    // library
    // Holonomic drive controller only relies on its current position, not its
    // velocity because the
    // target velocity is used as a ff
    // -------------------------------------------------------------------------------------
    PathPlannerTrajectoryState goal = trajectory.sample(Math.min(currentTime, trajectory.getTotalTimeSeconds()));
    // System.out.println("goallspeed:" + goal.linearVelocity);
    Trajectory.State state = new Trajectory.State(
        currentTime,
        goal.linearVelocity,
        0.0,
        new Pose2d(goal.pose.getTranslation(), goal.heading),
        0.0
    );

    // construct chassisspeeds
    adjustedSpeeds = hocontroller.calculate(currentPose, state, goal.pose.getRotation());
    // System.out.println("bv: " + Math.hypot(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond));

    adjustedSpeeds = applyCusp(adjustedSpeeds, translationError, CONVERGE_DISTANCE);

    // if(currentTime < 0.7){
    //   adjustedSpeeds = adjustedSpeeds.times(7);
    // }
    // System.out.println("av: " + Math.hypot(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond));

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

    // send to drivetrain
    m_driveTrain.drive(adjustedSpeeds);

    // Named Commands
    eventScheduler.execute(currentTime);

    // Logging
    debugLogsTrajectory();
  } // end of execute

  // ---------------------------------------------------------------------------------------------
  //
  // Debug Logs
  //
  // ---------------------------------------------------------------------------------------------
  public void debugLogsTrajectory() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("trajectory done");

    timer.stop(); // Stop timer
    m_driveTrain.stopDriving();

    PathPlannerAuto.currentPathName = "";
    PathPlannerAuto.setCurrentTrajectory(null);
    PathPlannerLogging.logActivePath(null);

    eventScheduler.end();
    if (interrupted) {
      System.out.println("OH NO I WAS INTERRUPTED HOW RUDE");
    }else{
      //Rumble the controller for NBA when auto aiming is done
      if(xboxDrv != null){
        xboxDrv.setRumble(RumbleType.kBothRumble, 0.7);
        Timer.delay(0.2);
        xboxDrv.setRumble(RumbleType.kBothRumble, 0.0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    // System.out.println("vision: "
    // +tracker.getDEstimatedPose().getTranslation().getNorm() );
    if(isBugged){
      System.out.println("Path Bugged");
      return true;
    }
    // Event Command or timeout
    if (timer.hasElapsed(pathTimeOut)) {
      System.out.println("timed out!!@)!*()*!)(#*)");
      return true;
    }
    // System.out.println("vision: " + (tracker.getVisionPoseShift().getNorm()) + " pose: " + translationError);

    if (container.getCatzVision().isSeeingApriltag() && autoalign && tracker.getVisionPoseShift().getNorm() > ALLOWABLE_VISION_ADJUST) {
      // If trailing pose is within margin
      // System.out.println("vision is not true");
      return false;
    }
    // Finish command if the total time the path takes is over

    // Autonomous vs Autoalign margins
    if (autoalign) {
      // System.out.println("passed vision check!!!@)*)(*)(@*#()*@)#(*)");
      return isAtGoalState(ALLOWABLE_AUTOAIM_ERROR);
    } else {
      // System.out.println("not auto align!@!@!");
      return isAtGoalState(ALLOWABLE_POSE_ERROR);
    }
  }

  private ChassisSpeeds applyCusp(ChassisSpeeds speeds, double distance, double threshold) {
    // graph 1 / (1+x) on desmos
    double x = distance / threshold;
    double omega = speeds.omegaRadiansPerSecond;

    ChassisSpeeds s = speeds.times(Math.min(2 * x / (1 + x), 1.0));
    s.omegaRadiansPerSecond = omega;
    return s;

  }

  public boolean isAtGoalState(double poseError){
    PathPlannerTrajectoryState endState = trajectory.getEndState();

    double currentRotation = tracker.getEstimatedPose().getRotation().getDegrees();
    double desiredRotation = endState.pose.getRotation().getDegrees();

    ChassisSpeeds currentChassisSpeeds = tracker.getRobotChassisSpeeds();
    double desiredMPS = trajectory.getEndState().linearVelocity;
    double currentMPS = Math.hypot(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);
    double currentRPS = Units.radiansToDegrees(currentChassisSpeeds.omegaRadiansPerSecond);

    double rotationError = Math.abs(desiredRotation - currentRotation);
    if (rotationError > 180) {
      rotationError = 360 - rotationError;
    }
    // System.out.println("rotationerr: " + (rotationError < ALLOWABLE_OMEGA_ERROR));
    // System.out.println("speederr: " + currentMPS);

    return isPoseWithinThreshold(poseError) && rotationError < ALLOWABLE_ROTATION_ERROR &&
    (desiredMPS != 0.0 || (currentMPS < ALLOWABLE_VEL_ERROR && currentRPS < ALLOWABLE_OMEGA_ERROR));
  }

  public boolean isPoseWithinThreshold(double poseError) {
    // Check if the robot is near goal (and if robot velocity is zero if goal
    // velocity is zero)
    PathPlannerTrajectoryState endState = trajectory.getEndState();

    double currentPosX = tracker.getEstimatedPose().getX();
    double currentPosY = tracker.getEstimatedPose().getY();

    double desiredPosX = endState.pose.getX();
    double desiredPosY = endState.pose.getY();

    double xError = Math.abs(desiredPosX - currentPosX);
    double yError = Math.abs(desiredPosY - currentPosY);

    translationError = Math.hypot(xError, yError);

    // System.out.println("poseerr:" + ((xError < poseError) &&(yError < poseError)));
    // System.out.println("transerr: " + translationError);
    // System.out.println("pose errr: " + poseError);
    return translationError < poseError;
  }
}
