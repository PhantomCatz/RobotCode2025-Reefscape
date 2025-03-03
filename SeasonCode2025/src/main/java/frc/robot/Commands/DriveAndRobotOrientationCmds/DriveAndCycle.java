// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.RobotContainer;
import frc.robot.TeleopPosSelector;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;

public class DriveAndCycle extends TrajectoryDriveCmd{
    private final double PREDICT_DISTANCE = 0.3; // meters

    private final RobotAction action;
    private TeleopPosSelector selector;
    private CatzSuperstructure superstructure;
    private CatzOuttake outtake;
    private int level;

    private boolean actionAlreadyTaken = false;

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action){
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE, container);
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        this.selector = container.getSelector();
        addRequirements(super.getRequirements());
    }

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action, int level){
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE, container);
        this.level = level;
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        this.selector = container.getSelector();
        addRequirements(super.getRequirements());
    }

    @Override
    public void initialize(){
        super.initialize();
        // selector.hasCoralSIM = true;
    }

    @Override
    public void execute(){
        // Run Trajectory
        if( super.isFinished() == false){
            super.execute();
        }

        // Run Scoring or Intaking
        if (super.isPoseWithinThreshold(PREDICT_DISTANCE) && !super.isFinished() && !actionAlreadyTaken){
            actionAlreadyTaken = true;
            if(action == RobotAction.OUTTAKE && superstructure.getCurrentRobotAction() != RobotAction.OUTTAKE){
                System.out.println("raised elevator!!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.AIMING, level);
            }
            else if(action == RobotAction.INTAKE){
                System.out.println("intaking!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.INTAKE, "intak");
            }
        }

        // If we reached the target Destination
        if (super.isFinished()){
            super.end(false);
            if(action == RobotAction.OUTTAKE){
                superstructure.setCurrentRobotAction(action, this.level);
            }
            // System.out.println("action: " + action.toString());
            if (selector.useFakeCoral){
                selector.hasCoralSIM = action == RobotAction.INTAKE;
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        if(action == RobotAction.OUTTAKE){
            System.out.println("Auto Stowing");
            superstructure.setCurrentRobotAction(RobotAction.STOW, "dnc end");
        }
    }

    @Override
    public boolean isFinished(){
        if(action == RobotAction.OUTTAKE){
            return !outtake.hasCoral();
        }
        if(action == RobotAction.INTAKE){
            return outtake.hasCoral();
        }
        return false;
    }
}
