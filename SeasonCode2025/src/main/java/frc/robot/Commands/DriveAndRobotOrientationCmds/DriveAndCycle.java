// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import java.util.Arrays;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;

public class DriveAndCycle extends TrajectoryDriveCmd {
    private final double PREDICT_DISTANCE = 0.3; // meters

    private final RobotAction action;
    private CatzSuperstructure superstructure;
    private CatzOuttake outtake;
    private int level;
    private RobotContainer container;
    private final Pose2d trueGoal;
    private Command driveForwardScoreCmd;

    private boolean actionAlreadyTaken = false; //i am sorry im currently writing this in my physics class and im running low on time :(
    private boolean alreadyOuttake = false;
    private boolean alreadyStopped = false;

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action) {
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE, container);
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        addRequirements(super.getRequirements());
        this.container = container;
        this.trueGoal = null;
    }

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action, int level,
            Pose2d trueGoal) {
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE, container);
        this.level = level;
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        addRequirements(super.getRequirements());
        this.container = container;
        this.trueGoal = trueGoal;
        this.driveForwardScoreCmd = getDriveForwardCommand();
    }

    @Override
    public void initialize() {
        super.initialize();
        // selector.hasCoralSIM = true;
        actionAlreadyTaken = false;
        alreadyOuttake = false;
    }

    @Override
    public void execute() {
        // Run Trajectory
        boolean initialDriveFinished = super.isFinished();
        if (initialDriveFinished == false) {
            super.execute();
        }

        // Run Scoring or Intaking
        if (super.isPoseWithinThreshold(PREDICT_DISTANCE) && !super.isFinished() && !actionAlreadyTaken) {
            actionAlreadyTaken = true;
            if (action == RobotAction.OUTTAKE && superstructure.getCurrentRobotAction() != RobotAction.OUTTAKE) {
                // System.out.println("raised elevator!!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.AIMING, level);
            } else if (action == RobotAction.INTAKE) {
                // System.out.println("intaking!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.INTAKE, "intak");
            }
        }

        // If we reached the target Destination
        if (initialDriveFinished && !alreadyStopped) {
            super.end(false);
            alreadyStopped = true;
            
        }

        if (initialDriveFinished && action == RobotAction.OUTTAKE && !alreadyOuttake) {
            alreadyOuttake = true;
            driveForwardScoreCmd.execute();
            // superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, this.level);

            if(driveForwardScoreCmd.isFinished()){
                if (container.getSelector().useFakeCoral) {
                    container.getSelector().hasCoralSIM = action == RobotAction.INTAKE;
                }
            }
        }
        // System.out.println("action: " + action.toString());
        
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (action == RobotAction.OUTTAKE) {
            if (level == 4) { // TODO not the best way to do it. eric already had code for it but i didnt have
                              // time to test so just ducttape fix
                Timer.delay(0.5);
            }
            superstructure.setCurrentRobotAction(RobotAction.STOW, "dnc end");
        }
    }

    @Override
    public boolean isFinished() {
        if (action == RobotAction.OUTTAKE) {
            return super.isFinished();
        }
        if (action == RobotAction.INTAKE) {
            return outtake.hasCoral();
        }
        return false;
    }

    //TODO make a function for creating a straight line trajectory
    private Command getDriveForwardCommand() {
        Pose2d currentPose = CatzRobotTracker.getInstance().getEstimatedPose();
        Translation2d direction = trueGoal.getTranslation().minus(currentPose.getTranslation()).div(2.0);
        if (direction.getNorm() <= 1e-3) {
            return null;
        }

        PathPlannerPath path = new PathPlannerPath(
                Arrays.asList(new Waypoint[] {
                        new Waypoint(null, currentPose.getTranslation(), currentPose.getTranslation().plus(direction)),
                        new Waypoint(trueGoal.getTranslation().minus(direction), trueGoal.getTranslation(), null)
                }), DriveConstants.LEFT_RIGHT_CONSTRAINTS, null, new GoalEndState(0.0, trueGoal.getRotation()));

        if(AllianceFlipUtil.shouldFlipToRed()){
            path = path.flipPath();
        }

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(path, container.getCatzDrivetrain(), true, container),
            new InstantCommand(() -> superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, level))
        );
    }
}
