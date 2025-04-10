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
package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Utilities.AllianceFlipUtil;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.function.Supplier;

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
  public static final double ALLOWABLE_AUTOAIM_ERROR = 0.025;
  public static final double ALLOWABLE_ROTATION_ERROR = 1.5;
  public static final double ALLOWABLE_VEL_ERROR = 0.80; // m/s
  public static final double ALLOWABLE_OMEGA_ERROR = 10.0;
  private static final double TIMEOUT_EXTRA = 2.0;
  private static final double CONVERGE_DISTANCE = 0.04;
  private static final double CONVERGE_ANGLE = 1.0;
  private static final double FACE_REEF_DIST = 2.0;
  private final double ALLOWABLE_VISION_ADJUST = 2e-3; //TODO tune

  // Subsystems
  private CatzDrivetrain m_driveTrain;
  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
  private final RobotContainer container;

  // Trajectory variables
  private HolonomicDriveController hocontroller;
  private PathPlannerTrajectory trajectory;
  private PathPlannerPath path;
  private Supplier<PathPlannerPath> pathSupplier;
  private Rotation2d endRotation;

  private double pathTimeOut = -999.0;
  private Timer timer = new Timer();
  private boolean autoalign = false;
  private PathPlannerTrajectoryState isTwoPathPoints = null;
  private PathPlannerTrajectoryState endState;
  private double translationError = FieldConstants.FIELD_LENGTH_MTRS * 2;

  // Swerve Drive Variables
  private ChassisSpeeds adjustedSpeeds = new ChassisSpeeds();

  private boolean isBugged = false;
  private final boolean isTelop;
  // ---------------------------------------------------------------------------------------------
  //
  // Trajectory Drive Command Constructor
  //
  // ---------------------------------------------------------------------------------------------
  public TrajectoryDriveCmd(PathPlannerPath newPath, CatzDrivetrain drivetrain, boolean autoalign, RobotContainer container, boolean isTeleop) {
    this.path = newPath;
    this.m_driveTrain = drivetrain;
    this.autoalign = autoalign;
    this.container = container;
    this.isTelop = isTeleop;
    addRequirements(m_driveTrain);
  }

  public TrajectoryDriveCmd(Supplier<PathPlannerPath> newPathSupplier, CatzDrivetrain drivetrain, boolean autoalign, RobotContainer container, boolean isTeleop) {
    this.pathSupplier = newPathSupplier;
    this.m_driveTrain = drivetrain;
    this.autoalign = autoalign;
    this.container = container;
    this.isTelop = isTeleop;
    addRequirements(m_driveTrain);
  }

  public void setPath(PathPlannerPath path){
    this.path = path;
  }
  private double prevTime;
  // ---------------------------------------------------------------------------------------------
  //
  // Initialize
  //
  // ---------------------------------------------------------------------------------------------
  void printTime(String tag){
    double curTime = Timer.getFPGATimestamp();
    // System.out.println(tag + (curTime-prevTime));
    prevTime = curTime;
  }
  @Override
  public void initialize() {
    prevTime = Timer.getFPGATimestamp();
    try{
      // Flip path if necessary
      PathPlannerPath usePath;
      if(pathSupplier != null) {
        path = pathSupplier.get();
        usePath = path;
      } else {
        usePath = path;
      }
      if(AllianceFlipUtil.shouldFlipToRed()) {
        usePath = path.flipPath();
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
      List<PathPoint> pathPoints = usePath.getAllPathPoints();


      boolean shouldChangeStartRot = tracker.getEstimatedPose().getTranslation().minus(pathPoints.get(pathPoints.size()-1).position).getNorm() > 0.50;
      Rotation2d startRot = tracker.getEstimatedPose().getRotation();

      if(shouldChangeStartRot){
        //if you are trying to auto align and the change in rotation is kind of big, then you want to set your starting rotation to the starting heading of path because for some mysterious reason
        //trajectory speed gets very slow when it needs to account for rotation. the only case where this wouldn't be auto align is if you are going to coral station
        //during autonomous, where you want to be facing is, so +180 for this case
        //i really dont like this fix either but i cant find the source of the issue so ¯\_(ツ)_/¯
        startRot = autoalign ? pathPoints.get(1).position.minus(pathPoints.get(0).position).getAngle() : pathPoints.get(1).position.minus(pathPoints.get(0).position).getAngle().plus(Rotation2d.k180deg);
      }

      try {
        // Construct trajectory
        this.trajectory = usePath.generateTrajectory(currentSpeeds, startRot, DriveConstants.TRAJ_ROBOT_CONFIG);
      } catch (Error e){
        e.printStackTrace();
        //for some reason if you spam NBA current rotation gets bugged.
        this.trajectory = usePath.generateTrajectory(currentSpeeds, new Rotation2d(), DriveConstants.TRAJ_ROBOT_CONFIG);
        // this.trajectory = new PathPlannerTrajectory(
          //   usePath,
          //   currentSpeeds, //TODO make it not zero if its a thing thingy y esdpoifi
          //   new Rotation2d(),
          //   DriveConstants.TRAJ_ROBOT_CONFIG
          // );
        }

        if(trajectory == null) {
          System.out.println("bugged!!!!!!!");
          isBugged = true;
          return;
        }

        if(pathPoints.size() == 2){

          Pose2d endPose = new Pose2d(pathPoints.get(1).position, usePath.getGoalEndState().rotation());
          isTwoPathPoints = new PathPlannerTrajectoryState();
          isTwoPathPoints.pose = endPose;
          endState = isTwoPathPoints;
        } else {
          endState = trajectory.sample(trajectory.getTotalTimeSeconds());
        }
        endRotation = endState.pose.getRotation();

      hocontroller = DriveConstants.getNewHolController();

      if(isTelop){
        pathTimeOut = 999999.0;
      }else{
        pathTimeOut = trajectory.getTotalTimeSeconds() + TIMEOUT_EXTRA;
      }


      // System.out.println("current " + tracker.getEstimatedPose());
      // System.out.println("start " + this.trajectory.getInitialPose());
      // System.out.println("end " + this.trajectory.getEndState().pose);

      // Reset
      PathPlannerLogging.logActivePath(usePath);
      PPLibTelemetry.setCurrentPath(usePath);

      this.timer.reset();
      this.timer.start();
    } catch(Exception e) {
      isBugged = true;
      e.printStackTrace();

    }

    // System.out.println("timeoutt::" + trajectory.getTotalTimeSeconds());
  } // end of initialize()



  // ---------------------------------------------------------------------------------------------
  //
  // Execute
  //
  // ---------------------------------------------------------------------------------------------
  private double exeTime = 0.0;
  @Override
  public void execute() {
    if(this.trajectory == null || isBugged) return;
    exeTime = Timer.getFPGATimestamp();

    // Collect instananous variables
    double currentTime = timer.get();
    Pose2d currentPose = tracker.getEstimatedPose();
    // ChassisSpeeds currentSpeeds = DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(m_driveTrain.getModuleStates());
    // double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

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
    if(isTwoPathPoints != null){
      goal = isTwoPathPoints;
    }

    Trajectory.State state = new Trajectory.State(
        currentTime,
        goal.linearVelocity,
        0.0,
        new Pose2d(goal.pose.getTranslation(), goal.heading),
        0.0
    );


    // construct chassisspeeds
    adjustedSpeeds = hocontroller.calculate(currentPose, state, endRotation);
    // if (autoalign && translationError > FACE_REEF_DIST) {
    //   Translation2d reef = AllianceFlipUtil.apply(FieldConstants.Reef.center);
    //   adjustedSpeeds = hocontroller.calculate(currentPose, state, Rotation2d.fromRadians(Math.atan2(reef.getY() - currentPose.getY(),reef.getX() - currentPose.getX())));
    // }else{
    // }

    adjustedSpeeds = applyCusp(adjustedSpeeds, translationError, endRotation.minus(currentPose.getRotation()).getDegrees(), CONVERGE_DISTANCE, CONVERGE_ANGLE);

    // Logging
    Logger.recordOutput("CatzRobotTracker/Desired Auto Pose", goal.pose);

    // send to drivetrain
    m_driveTrain.drive(adjustedSpeeds);

    double currentPosX = tracker.getEstimatedPose().getX();
    double currentPosY = tracker.getEstimatedPose().getY();

    double desiredPosX = endState.pose.getX();
    double desiredPosY = endState.pose.getY();

    double xError = Math.abs(desiredPosX - currentPosX);
    double yError = Math.abs(desiredPosY - currentPosY);

    translationError = Math.hypot(xError, yError);

    m_driveTrain.setDistanceError(translationError);

    CatzRobotTracker.getInstance().setCoralStationTrajectoryRemaining(trajectory.getTotalTimeSeconds()-currentTime);

    // Logging
    // debugLogsTrajectory();

    // System.out.println("traj exe: " + (Timer.getFPGATimestamp() - exeTime));
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
    m_driveTrain.drive(new ChassisSpeeds());
    Logger.recordOutput("CatzRobotTracker/Desired Auto Pose", new Pose2d());

    // PathPlannerAuto.currentPathName = "";
    // Logger.recordOutput("CatzRobotTracker/Desired Auto Pose", new Pose2d());

    if (interrupted) {
      System.out.println("OH NO I WAS INTERRUPTED HOW RUDE");
    }

    m_driveTrain.setDistanceError(9999999.9);
  }


  @Override
  public boolean isFinished() {
    if(isBugged){
      System.out.println("Path Bugged");
      return true;
    }
    // Event Command or timeout
    if (timer.hasElapsed(pathTimeOut)) {
      System.out.println("path timed out!");
      return true;
    }

    // dont end prematurely
    if(!timer.hasElapsed(trajectory.getTotalTimeSeconds())){
      return false;
    }

    System.out.println("vision: " + tracker.getVisionPoseShift().getNorm());
    if (container.getCatzVision().isSeeingApriltag() && autoalign && tracker.getVisionPoseShift().getNorm() > ALLOWABLE_VISION_ADJUST) {
      // If trailing pose is within margin
      System.out.println("not visioning");
      return false;
    }
    // Finish command if the total time the path takes is over

    // Autonomous vs Autoalign margins
    if (autoalign) {
      return isAtGoalState(ALLOWABLE_AUTOAIM_ERROR);
    } else {
      return isAtGoalState(ALLOWABLE_POSE_ERROR);
    }
  }

  //---------------------------------------------------------------------------------------------------------------
  //  Trajectory Factory Methods
  //---------------------------------------------------------------------------------------------------------------
  private ChassisSpeeds applyCusp(ChassisSpeeds speeds, double distance, double angle, final double thresholdDist, final double thresholdAngle) {
    // graph 1 / (1+x) on desmos
    double x = distance / thresholdDist;
    double omega = speeds.omegaRadiansPerSecond;

    double x2 = angle / thresholdAngle;
    omega = omega * Math.min(2*x2 / (1+x2), 1.0);

    ChassisSpeeds s = speeds.times(Math.min(2 * x / (1 + x), 1.0));

    s.omegaRadiansPerSecond = omega;
    return s;

  }

  public boolean isAtGoalState(double poseError){
    double currentRotation = tracker.getEstimatedPose().getRotation().getDegrees();
    double desiredRotation = endState.pose.getRotation().getDegrees();

    ChassisSpeeds currentChassisSpeeds = tracker.getRobotChassisSpeeds();
    double desiredMPS = endState.linearVelocity;
    double currentMPS = Math.hypot(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);
    double currentRPS = Units.radiansToDegrees(currentChassisSpeeds.omegaRadiansPerSecond);

    double rotationError = Math.abs(desiredRotation - currentRotation);
    if (rotationError > 180) {
      rotationError = 360 - rotationError;
    }

    // Desperate Throwing
    if(Robot.getAutoElapsedTime() >= 14.50) {
      rotationError = 0.0;
      System.out.println("THROW");
    }

    // System.out.println("rotationerr: " + (rotationError));
    // System.out.println("omegaerr: " + (currentRPS));
    // System.out.println("speederr: " + (currentMPS));

    CatzRobotTracker.getInstance().setReachedGoal(isPoseWithinThreshold(poseError));

    return isPoseWithinThreshold(poseError) && rotationError < ALLOWABLE_ROTATION_ERROR &&
    (desiredMPS != 0.0 || (currentMPS < ALLOWABLE_VEL_ERROR && currentRPS < ALLOWABLE_OMEGA_ERROR));
  }

  public boolean isPoseWithinThreshold(double poseError) {
    return translationError < poseError;
  }
}
