package frc.robot.Commands.DriveAndRobotOrientationCmds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

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
    private PathPlannerTrajectory trajectory;
    private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
    
    // Trajectory variables
    private HolonomicDriveController hocontroller;
    private PathPlannerPath path;
    private boolean atTarget = false;
    private double pathTimeOut = -999.0;
    private Timer timer = new Timer();

    // Event Command variables
    private final Map<Command, Boolean> currentEventCommands = new HashMap();
    private final List<Pair<Double, Command>> untriggeredEvents = new ArrayList();
    private boolean isEventCommandRunning = false;

    //Constructor Logger
    private int m_constructorLogger = 1; // For determining if the command is auto path find or autonomous

    //---------------------------------------------------------------------------------------------
    //
    // Trajectory Drive Command Constructor
    // 
    //---------------------------------------------------------------------------------------------
    public TrajectoryDriveCmd(PathPlannerPath newPath, CatzDrivetrain drivetrain) {
        path = newPath;
        m_driveTrain = drivetrain;
        addRequirements(m_driveTrain);

        Iterator var10 = this.path.getEventMarkers().iterator();
        while(var10.hasNext()) {
            EventMarker marker = (EventMarker)var10.next();
            Set<Subsystem> reqs = marker.getCommand().getRequirements();
            this.m_requirements.addAll(reqs);
        }
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
        if(AllianceFlipUtil.shouldFlipToRed()) {
            usePath = path.flipPath();
        }

        // Construct trajectory
        this.trajectory = new PathPlannerTrajectory(
            usePath, 
            DriveConstants.
                swerveDriveKinematics.
                    toChassisSpeeds(tracker.getCurrentModuleStates()),
            tracker.getEstimatedPose().getRotation()
        );

        hocontroller = DriveConstants.getNewHolController();                                
        pathTimeOut = trajectory.getTotalTimeSeconds() * TIMEOUT_SCALAR; //TODO do we still need this

        // Reset
        this.currentEventCommands.clear();
        this.untriggeredEvents.clear();
        this.untriggeredEvents.addAll(this.trajectory.getEventCommands());
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
        double currentTime = this.timer.get();

        // Trajectory Executor
        if(!atTarget){
            // Getters from pathplanner and current robot pose
            PathPlannerTrajectory.State goal = trajectory.sample(Math.min(currentTime, trajectory.getTotalTimeSeconds()));
            Rotation2d targetOrientation     = goal.targetHolonomicRotation;
            Pose2d currentPose               = tracker.getEstimatedPose();
                
            //-------------------------------------------------------------------------------------
            // Convert PP trajectory into a wpilib trajectory type 
            // Only takes in the current robot position 
            // Does not take acceleration to be used with the internal WPILIB trajectory library
            // Holonomic drive controller only relies on its current position, not its velocity because the target velocity is used as a ff
            //-------------------------------------------------------------------------------------
            Trajectory.State state = new Trajectory.State(
                currentTime, 
                goal.velocityMps,  //made the 
                goal.accelerationMpsSq, 
                new Pose2d(goal.positionMeters, goal.heading),
                goal.curvatureRadPerMeter
            );
    
            //construct chassisspeeds
            ChassisSpeeds adjustedSpeeds = hocontroller.calculate(currentPose, state, targetOrientation);

            //send to drivetrain
            m_driveTrain.drive(adjustedSpeeds);
            tracker.addTrajectorySetpointData(goal.getTargetHolonomicPose());

            // Log desired pose
            Logger.recordOutput("CatzRobotTracker/Desired Auto Pose", new Pose2d(state.poseMeters.getTranslation(), goal.targetHolonomicRotation));
        }
        else{
            m_driveTrain.stopDriving();
        } // End of if(!atTarget) else
        
        //-------------------------------------------------------------------------------------
        // TODO how does this work?
        //-------------------------------------------------------------------------------------
        if (!this.untriggeredEvents.isEmpty() && this.timer.hasElapsed((Double)((Pair)this.untriggeredEvents.get(0)).getFirst())) {
            Pair<Double, Command> event = (Pair)this.untriggeredEvents.remove(0);
            Iterator var10 = this.currentEventCommands.entrySet().iterator();
   
            while(var10.hasNext()) {
               Map.Entry<Command, Boolean> runningCommand = (Map.Entry)var10.next();
               if ((Boolean)runningCommand.getValue() && !Collections.disjoint(((Command)runningCommand.getKey()).getRequirements(), ((Command)event.getSecond()).getRequirements())) {
                  ((Command)runningCommand.getKey()).end(true);
                  runningCommand.setValue(false);
               }
            }
   
            ((Command)event.getSecond()).initialize();
            this.currentEventCommands.put((Command)event.getSecond(), true);
            isEventCommandRunning = true;
         } // end of if (!this.untriggeredEvents.isEmpty())
   
        //-------------------------------------------------------------------------------------
        // TODO how does this work?
        //-------------------------------------------------------------------------------------
        Iterator var13 = this.currentEventCommands.entrySet().iterator();
        while(var13.hasNext()) {
            Map.Entry<Command, Boolean> runningCommand = (Map.Entry)var13.next();
            if ((Boolean)runningCommand.getValue()) {
                ((Command)runningCommand.getKey()).execute();
                if (((Command)runningCommand.getKey()).isFinished()) {
                    ((Command)runningCommand.getKey()).end(false);
                    runningCommand.setValue(false);
                    isEventCommandRunning = false;
                }
            }
        } // end of while(var13.hasNext()) 
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
    }

    @Override
    public boolean isFinished() {
        // Finish command if the total time the path takes is over
        State endState = trajectory.getEndState();

        double currentPosX =        tracker.getEstimatedPose().getX();
        double currentPosY =        tracker.getEstimatedPose().getY();
        double currentRotation =    tracker.getEstimatedPose().getRotation().getDegrees();

        double desiredPosX =        endState.positionMeters.getX();
        double desiredPosY =        endState.positionMeters.getY();
        double desiredRotation =    endState.targetHolonomicRotation.getDegrees();

        //Another condition to end trajectory. If end target velocity is zero, then only stop if the robot velocity is also near zero so it doesn't run over its target.
        double desiredMPS = trajectory.getEndState().velocityMps;
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