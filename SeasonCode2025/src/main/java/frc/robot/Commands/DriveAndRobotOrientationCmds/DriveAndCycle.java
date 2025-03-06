// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
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
    private Command endCommand;
    private int level;

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action){
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE);
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.selector = container.getSelector();
        addRequirements(super.getRequirements());
    }

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action, int level){
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE);
        this.level = level;
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.selector = container.getSelector();
        addRequirements(super.getRequirements());
    }

    @Override
    public void initialize(){
        super.initialize();
        selector.hasCoralSIM = true;
    }

    private boolean isAiming = false;

    @Override
    public void execute(){
        // Run Trajectory
        if(!super.isFinished()){
            super.execute();
        }

        // Run Scoring or Intaking
        if (super.isPoseWithinThreshold(PREDICT_DISTANCE) && !super.isFinished() && !isAiming){
            if(action == RobotAction.OUTTAKE){
                superstructure.setCurrentRobotAction(RobotAction.AIMING, level);
            }
            else if(action == RobotAction.INTAKE){
                superstructure.setCurrentRobotAction(RobotAction.INTAKE);
            }
            isAiming = true;
        }

        // If we reached the target Destination
        if (super.isFinished() && isAiming){
            super.end(false);
            if (selector.useFakeCoral){
                selector.hasCoralSIM = action == RobotAction.INTAKE;
            }

            Command scheduledCommand = superstructure.setCurrentRobotAction(action, this.level);
            if(scheduledCommand != null){
                endCommand = scheduledCommand;
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
        if(endCommand != null){
            return endCommand.isFinished();
        }
        return false;
    }
}
