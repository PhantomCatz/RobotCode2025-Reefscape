// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.CatzSubsystems.CatzSuperstructure.Gamepiece;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;

public class DriveToScore extends TrajectoryDriveCmd{
    private final double PREDICT_DISTANCE = 0.3; // meters

    private final RobotAction action = RobotAction.OUTTAKE;
    private CatzSuperstructure superstructure;
    private CatzOuttake outtake;
    private int level;
    private RobotContainer container;

    private boolean actionAlreadyTaken = false;
    private boolean alreadyOuttake = false;
    private boolean skipped = false;

    public DriveToScore(PathPlannerPath newPath, RobotContainer container){
        super(newPath, container.getCatzDrivetrain(), true, container);
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        addRequirements(super.getRequirements());
        this.container = container;
    }

    public DriveToScore(PathPlannerPath newPath, RobotContainer container, int level){
        super(newPath, container.getCatzDrivetrain(), true, container);
        this.level = level;
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        addRequirements(super.getRequirements());
        this.container = container;
    }

    @Override
    public void initialize(){
        super.initialize();
        // selector.hasCoralSIM = true;
        actionAlreadyTaken = false;
        alreadyOuttake = false;
        skipped = false;
        CatzSuperstructure.setChosenGamepiece(Gamepiece.CORAL);
    }

    @Override
    public void execute(){
        // Run Trajectory
        if(super.isFinished() == false){
            super.execute();
        }

        // Run Preaiming
        // If we are within the threshold distance and we haven't already set the action
        // and we are not finished yet, set the action to aiming
        if (super.isPoseWithinThreshold(PREDICT_DISTANCE) && !super.isFinished() && !actionAlreadyTaken){
            actionAlreadyTaken = true;
            if(superstructure.getCurrentRobotAction() != RobotAction.OUTTAKE){
                System.out.println("raised elevator!!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.AIMING, level);
            }
        }

        // If we reached the target Destination
        if (super.isFinished()) {
            if(action == RobotAction.OUTTAKE && !alreadyOuttake){
                alreadyOuttake = true;
                superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, this.level);
            }

            // End Trajectory Command
            super.end(false);
        }
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        if(action == RobotAction.OUTTAKE){
            if(level == 4){ //TODO not the best way to do it. eric already had code for it but i didnt have time to test so just ducttape fix
               // Timer.delay(0.5);
            }
            superstructure.setCurrentRobotAction(RobotAction.STOW, "dnc end");
        }
    }

    @Override
    public boolean isFinished(){
            return !outtake.isDesiredCoralState(true);
    }
}
