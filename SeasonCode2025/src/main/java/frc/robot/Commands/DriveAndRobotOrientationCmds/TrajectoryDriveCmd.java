package frc.robot.Commands.DriveAndRobotOrientationCmds;

import static edu.wpi.first.units.Units.Ounce;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.Utilities.AllianceFlipUtil;

/**************************************************************************************************

* 

* TrajectoryDriveCmd

* 
 
**************************************************************************************************/

public class TrajectoryDriveCmd extends Command {
    // Trajectory constants
    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = 5;
    public static final double ALLOWABLE_VEL_ERROR = 0.2;
    private static final double TIMEOUT_SCALAR = 5;

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

    // Event Command variables
    private final EventScheduler eventScheduler;
    private final Map<Command, Boolean> currentEventCommands = new HashMap();
    private final List<Pair<Double, Command>> untriggeredEvents = new ArrayList();
    private boolean isEventCommandRunning = false;

    //---------------------------------------------------------------------------------------------
    //
    // Trajectory Drive Command Constructor
    // 
    //---------------------------------------------------------------------------------------------
    public TrajectoryDriveCmd(PathPlannerPath newPath, CatzDrivetrain drivetrain) {
        this.path = newPath;
        this.m_driveTrain = drivetrain;
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

    //---------------------------------------------------------------------------------------------
    //
    // Initialize
    // 
    //---------------------------------------------------------------------------------------------
    @Override
    public void initialize() {
        // Flip path if necessary
        PathPlannerPath usePath = path;
        Rotation2d flippedRotation2d = tracker.getEstimatedPose().getRotation();
        System.out.println(flippedRotation2d.getDegrees());
        if(AllianceFlipUtil.shouldFlipToRed()) {
            usePath = path.flipPath();
            flippedRotation2d = flippedRotation2d.plus(Rotation2d.fromDegrees(180));
        }
        
        // Construct trajectory
        this.trajectory = new PathPlannerTrajectory(
            usePath,
            DriveConstants.
                SWERVE_KINEMATICS
                    .toChassisSpeeds(tracker.getCurrentModuleStates()),
            flippedRotation2d,
            DriveConstants.TRAJECTORY_CONFIG
        );

        hocontroller = DriveConstants.getNewHolController(); 
        ppHoController = DriveConstants.getNewPathFollowingController();
        pathTimeOut = trajectory.getTotalTimeSeconds() * TIMEOUT_SCALAR; //TODO do we still need this

        // Reset
        PathPlannerLogging.logActivePath(usePath);
        PPLibTelemetry.setCurrentPath(usePath);

        eventScheduler.initialize(trajectory);
        this.timer.reset();
        this.timer.start();
    } //end of initialize()

    //---------------------------------------------------------------------------------------------
    //
    // Execute
    // 
    //---------------------------------------------------------------------------------------------
    @Override
    public void execute() {
        System.out.println("executing");
        double currentTime = this.timer.get();
        // Getters from pathplanner and current robot pose
        PathPlannerTrajectoryState goal = trajectory.sample(Math.min(currentTime, trajectory.getTotalTimeSeconds()));
        Rotation2d targetOrientation    = goal.heading;
        Pose2d currentPose              = tracker.getEstimatedPose();
        ChassisSpeeds currentSpeeds     = DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(m_driveTrain.getModuleStates());
        double currentVel               = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        System.out.println(atTarget);
        // Trajectory Executor
        if(!atTarget){
                
            //-------------------------------------------------------------------------------------
            // Convert PP trajectory into a wpilib trajectory type 
            // Only takes in the current robot position 
            // Does not take acceleration to be used with the internal WPILIB trajectory library
            // Holonomic drive controller only relies on its current position, not its velocity because the target velocity is used as a ff
            //-------------------------------------------------------------------------------------
            Trajectory.State state = new Trajectory.State(
                currentTime, 
                goal.linearVelocity,  //made the 
                goal.linearVelocity/goal.timeSeconds, //TODO verify if this does what we want it to do 
                goal.pose,
                0.0
            );
    
            //construct chassisspeeds
            ChassisSpeeds adjustedSpeeds = hocontroller.calculate(currentPose, state, targetOrientation);
            ChassisSpeeds ppAdjustedSpeeds = ppHoController.calculateRobotRelativeSpeeds(currentPose, goal);


            //send to drivetrain
            m_driveTrain.drive(adjustedSpeeds);
            tracker.addTrajectorySetpointData(goal.pose);
            System.out.println("hi");
            // Log desired pose
            Logger.recordOutput("CatzRobotTracker/Desired Auto Pose", new Pose2d(state.poseMeters.getTranslation(), goal.heading));
        }
        else{
            m_driveTrain.stopDriving();
        } // End of if(!atTarget) else
        

        //---------------------------------------------------------------------------------------------------------------------------
        // Named Commands
        //---------------------------------------------------------------------------------------------------------------------------
        eventScheduler.execute(currentTime);

        //---------------------------------------------------------------------------------------------------------------------------
        //  Logging
        //---------------------------------------------------------------------------------------------------------------------------
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
    } //end of execute

    //---------------------------------------------------------------------------------------------
    //
    // Debug Logs 
    // 
    //---------------------------------------------------------------------------------------------
    public void debugLogsTrajectory(){
        //Logger.recordOutput("Desired Auto Pose", new Pose2d(state.poseMeters.getTranslation(), goal.targetHolonomicRotation));
        //Logger.recordOutput("Adjusted Speeds X", adjustedSpeeds.vxMetersPerSecond);
        //Logger.recordOutput("Adjusted Speeds Y", adjustedSpeeds.vyMetersPerSecond);
        //Logger.recordOutput("Trajectory Goal MPS", state.velocityMetersPerSecond);
        //Logger.recordOutput("PathPlanner Goal MPS", goal.velocityMps);

        //System.out.println(goal.getTargetHolonomicPose());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop(); // Stop timer
        m_driveTrain.stopDriving();
        System.out.println("trajectory done");

        timer.stop();
        PathPlannerAuto.currentPathName = "";
        PathPlannerAuto.setCurrentTrajectory(null);
        PathPlannerLogging.logActivePath(null);

        eventScheduler.end();
    }

    @Override
    public boolean isFinished() {
        // Finish command if the total time the path takes is over
        PathPlannerTrajectoryState endState = trajectory.getEndState();

        double currentPosX =        tracker.getEstimatedPose().getX();
        double currentPosY =        tracker.getEstimatedPose().getY();
        double currentRotation =    tracker.getEstimatedPose().getRotation().getDegrees();

        double desiredPosX =        endState.pose.getX();
        double desiredPosY =        endState.pose.getY();
        double desiredRotation =    endState.pose.getRotation().getDegrees();

        //Another condition to end trajectory. If end target velocity is zero, then only stop if the robot velocity is also near zero so it doesn't run over its target.
        double desiredMPS = trajectory.getEndState().linearVelocity;
        ChassisSpeeds currentChassisSpeeds = tracker.getRobotChassisSpeeds();
        double currentMPS = Math.hypot(Math.hypot(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond), currentChassisSpeeds.omegaRadiansPerSecond);

        double xError =        Math.abs(desiredPosX - currentPosX);
        double yError =        Math.abs(desiredPosY - currentPosY);
        double rotationError = Math.abs(desiredRotation - currentRotation);
        if (rotationError > 180){
            rotationError = 360-rotationError;
        }

        atTarget = (
            xError < ALLOWABLE_POSE_ERROR && 
            yError < ALLOWABLE_POSE_ERROR && 
            rotationError < ALLOWABLE_ROTATION_ERROR &&
            (desiredMPS == 0 || currentMPS < ALLOWABLE_VEL_ERROR)
        );

        return atTarget || timer.hasElapsed(pathTimeOut) && !isEventCommandRunning;
    } 

}