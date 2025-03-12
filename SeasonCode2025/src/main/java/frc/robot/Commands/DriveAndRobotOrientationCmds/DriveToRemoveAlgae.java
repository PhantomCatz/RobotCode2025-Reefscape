// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class DriveToRemoveAlgae extends TrajectoryDriveCmd{
    private final double PREDICT_DISTANCE =0.05;// 0.3; // meters

    private final RobotAction action = RobotAction.INTAKE;
    private CatzSuperstructure superstructure;
    private int level = 0;
    private RobotContainer container;


    private boolean actionAlreadyTaken = false;


    public DriveToRemoveAlgae(PathPlannerPath newPath, RobotContainer container){
        super(newPath, container.getCatzDrivetrain(), false, container);
        this.superstructure = container.getSuperstructure();
        addRequirements(super.getRequirements());
        this.container = container;
    }

    public DriveToRemoveAlgae(PathPlannerPath newPath, RobotContainer container, int level){
        super(newPath, container.getCatzDrivetrain(), false, container);
        this.level = level;
        this.superstructure = container.getSuperstructure();
        addRequirements(super.getRequirements());
        this.container = container;
    }

    @Override
    public void initialize(){
        super.initialize();
        // selector.hasCoralSIM = true;
        actionAlreadyTaken = false;
    }

    @Override
    public void execute(){
        // Run Trajectory
        if( super.isFinished() == false){
            super.execute();
        }

        // Run Scoring or Intaking
        if (super.isPoseWithinThreshold(PREDICT_DISTANCE) && !actionAlreadyTaken){
            actionAlreadyTaken = true;
                System.out.println("intaking!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.INTAKE, "intak");
        }

        // If we reached the target Destination
        if (super.isFinished()){

            if (container.getSelector().useFakeCoral){
                container.getSelector().hasCoralSIM = action == RobotAction.INTAKE;
            }
            container.getAlgaePivot().AlgaePivot_Punch().schedule();

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
        return superstructure.getRobotActionCommand().isFinished();
    }

    private double timeElapsedSince(double time){
        return Timer.getFPGATimestamp() - time;
    }
}
