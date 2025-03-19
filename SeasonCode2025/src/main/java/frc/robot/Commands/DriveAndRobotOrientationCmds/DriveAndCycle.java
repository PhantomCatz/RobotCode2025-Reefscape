//------------------------------------------------------------------------------------
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;

public class DriveAndCycle extends TrajectoryDriveCmd{
    private final double PREDICT_DISTANCE =0.05;// 0.3; // meters

    private final RobotAction action;
    private CatzSuperstructure superstructure;
    private CatzOuttake outtake;
    private CatzRampPivot pivot;
    private int level = 0;
    private RobotContainer container;

    private int intakeIterationCounter = 0;

    private boolean actionAlreadyTaken = false;
    private boolean alreadyOuttake = false;
    private boolean skipped = false;

    private final double INTAKE_TIMEOUT = 2.0; //seconds
    private final double INTAKE_UNJAM_TIME = 0.2; //seconds
    private boolean isUnjamming = false;

    private double intake_time = 0.0;
    private double unjam_time = 0.0;

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action){
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE, container);
        this.action = action;
        this.superstructure = container.getSuperstructure();
        this.pivot = container.getCatzRampPivot();
        this.outtake = container.getCatzOuttake();
        addRequirements(super.getRequirements());
        this.container = container;
    }

    public DriveAndCycle(PathPlannerPath newPath, RobotContainer container, RobotAction action, int level){
        super(newPath, container.getCatzDrivetrain(), action != RobotAction.INTAKE, container);
        this.level = level;
        this.action = action;
        this.pivot = container.getCatzRampPivot();
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
        intakeIterationCounter = 0;
        skipped = false;
        intake_time = 0.0;
    }

    @Override
    public void execute(){
        // if(intake_time != 0.0 && (timeElapsedSince(intake_time) >= INTAKE_TIMEOUT)){
        //     isUnjamming = true;
        //     unjam_time = Timer.getFPGATimestamp();
        // }

        // if(isUnjamming){
        //     if(timeElapsedSince(unjam_time) < INTAKE_UNJAM_TIME){
        //         outtake.setCurrentState(outtakeStates.RAMP_EJECT);
        //     }else{
        //         isUnjamming = false;
        //         actionAlreadyTaken = false;
        //         intake_time = 0.0;
        //     }
        // }

        // Run Trajectory
        if( super.isFinished() == false){
            super.execute();
        }

        // Run Scoring or Intaking
        if (super.isPoseWithinThreshold(PREDICT_DISTANCE) && !actionAlreadyTaken){
            actionAlreadyTaken = true;
            if(action == RobotAction.OUTTAKE && superstructure.getCurrentRobotAction() != RobotAction.OUTTAKE){
                System.out.println("raised elevator!!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.AIMING, level);
            }
            else if(action == RobotAction.INTAKE){
                System.out.println("intaking!!!!!!");
                superstructure.setCurrentRobotAction(RobotAction.INTAKE, "intak");
                // intakeIterationCounter++;
                // if(!outtake.hasCoral() && intakeIterationCounter > 100) {
                //     skipped = true;
                // }
            }
        }

        // If we reached the target Destination
        if (super.isFinished()){
            super.end(false);
            if(action == RobotAction.OUTTAKE){
                alreadyOuttake = true;
                if(level > 0){
                    // superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, level);
                    superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, level);
                    // System.out.println("auto l4 score!!");
                } else {
                    superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, "DriveAndCycle");
                }

            }

            // if(intake_time == 0.0 && action == RobotAction.INTAKE){
            //     intake_time = Timer.getFPGATimestamp();
            // }
            // System.out.println("action: " + action.toString());
            if (container.getSelector().useFakeCoral){
                container.getSelector().hasCoralSIM = action == RobotAction.INTAKE;
            }
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
        if(action == RobotAction.OUTTAKE){
            return outtake.isDesiredCoralState(true);
        }
        if(action == RobotAction.INTAKE){
            return outtake.isDesiredCoralState(false);
        }
        return false;
    }

    private double timeElapsedSince(double time){
        return Timer.getFPGATimestamp() - time;
    }
}
