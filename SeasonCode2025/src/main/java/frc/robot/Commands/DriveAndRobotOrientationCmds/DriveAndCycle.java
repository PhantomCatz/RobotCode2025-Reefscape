// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.CatzSubsystems.CatzSuperstructure.CoralState;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.RobotContainer;
import frc.robot.TeleopPosSelector;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;

public class DriveAndCycle extends TrajectoryDriveCmd{
    private double PREDICT_DISTANCE = 1.0; // meters

    private RobotAction action;
    private TeleopPosSelector selector;
    private CatzSuperstructure superstructure;
    private CatzOuttake outtake;
    private int level;

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action){
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE);
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.outtake = container.getCatzOuttake();
        this.selector = container.getSelector();
        addRequirements(super.getRequirements());
    }

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action, int level){
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE);
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
        selector.hasCoralSIM = true;
    }

    @Override
    public void execute(){
        // Run Trajectory
        super.execute();

        // Run Scoring or Intaking
        if (super.isPoseWithinThreshold(PREDICT_DISTANCE) && !super.isFinished()){
            if(action == RobotAction.OUTTAKE){
                System.out.println("raised elevator!!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.AIMING, level);
            }
            else if(action == RobotAction.INTAKE){
                System.out.println("intaking!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.INTAKE);
            }
        }

        // If we reached the target Destination
        if (super.isFinished()){
            superstructure.setCurrentRobotAction(action, this.level);
            System.out.println("action: " + action.toString());
            if (selector.useFakeCoral){
                selector.hasCoralSIM = action == RobotAction.INTAKE;
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        if(action == RobotAction.OUTTAKE){
            superstructure.setCurrentRobotAction(RobotAction.STOW);
        }
    }

    @Override
    public boolean isFinished(){
        if(action == RobotAction.OUTTAKE){
            if(selector.useFakeCoral){
                return !selector.hasCoralSIM;
            }else{
                return (CatzSuperstructure.getCurrentCoralState() == CoralState.NOT_IN_OUTTAKE);
            }
        }
        if(action == RobotAction.INTAKE){
            if(selector.useFakeCoral){
                return selector.hasCoralSIM;
            }
            return (CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE);
        }
        return false;
    }
}
