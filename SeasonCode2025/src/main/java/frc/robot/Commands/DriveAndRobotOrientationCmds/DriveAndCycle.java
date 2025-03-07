// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
    private Pose2d trueGoal;
    private Command driveForwardScoreCmd;

    private boolean actionAlreadyTaken = false; //i am sorry im currently writing this in my physics class and im running low on time :(
    private boolean alreadyStopped = false;
    private boolean driveForwardAlreadyStarted = false;

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action) {
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE, container);
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        addRequirements(super.getRequirements());
        this.container = container;
        this.trueGoal = null;
        driveForwardScoreCmd = new InstantCommand();
    }

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action, int level, Pose2d trueGoal) {
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE, container);
        this.level = level;
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        addRequirements(super.getRequirements());
        this.container = container;
        this.trueGoal = trueGoal;
        driveForwardScoreCmd = new InstantCommand();
    }

    @Override
    public void initialize() {
        super.initialize();
        // selector.hasCoralSIM = true;
        actionAlreadyTaken = false;
        alreadyStopped = false;
        driveForwardAlreadyStarted = false;

        System.out.println("time for a new day. do you have coral? " + outtake.hasCoral());
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
            System.out.println("time to score");
            super.end(false);
            alreadyStopped = true;
            driveForwardScoreCmd = getDriveForwardCommand();
        }

        if (alreadyStopped && action == RobotAction.OUTTAKE) {
            if(!driveForwardAlreadyStarted){
                driveForwardScoreCmd.initialize();
                driveForwardAlreadyStarted = true;
            }

            driveForwardScoreCmd.execute();
            // superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, this.level);
        }
        // System.out.println("action: " + action.toString());

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("the day has ended" + interrupted + action);
        super.end(interrupted);

        driveForwardScoreCmd.end(interrupted);

        if (action == RobotAction.OUTTAKE) {
            if (level == 4) { // TODO not the best way to do it. eric already had code for it but i didnt have
                              // time to test so just ducttape fix
                // Timer.delay(0.5);
            }
            superstructure.setCurrentRobotAction(RobotAction.STOW, "dnc end");
        }
    }

    @Override
    public boolean isFinished() {
        if (action == RobotAction.OUTTAKE) {
            return !outtake.hasCoral();
        }
        if (action == RobotAction.INTAKE) {
            System.out.println("bbut do you have a coral?"+ outtake.hasCoral());
            return outtake.hasCoral();
        }
        return false;
    }

    //TODO make a function for creating a straight line trajectory
    private Command getDriveForwardCommand() {
        Pose2d currentPose = CatzRobotTracker.getInstance().getEstimatedPose();

        //if it is autonomous, the goal position was not flipped properly, so flip it here.
        if(DriverStation.isAutonomousEnabled()){
            trueGoal = AllianceFlipUtil.apply(trueGoal);
        }
        Command trajCommand = container.getSelector().getStraightLineTrajectory(currentPose, trueGoal, DriveConstants.LEFT_RIGHT_CONSTRAINTS, true);

        return new SequentialCommandGroup(
            trajCommand.alongWith(new PrintCommand("trajjajajaja")),
            new InstantCommand(() -> superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, level)).alongWith(new PrintCommand("strucucucu"))
        );
    }
}
