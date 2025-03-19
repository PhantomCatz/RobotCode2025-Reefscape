//------------------------------------------------------------------------------------
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.CatzSubsystems.CatzSuperstructure.Gamepiece;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;

public class DriveToCoralStation extends TrajectoryDriveCmd{
    private final double PREDICT_DISTANCE = 0.3; // meters

    private final RobotAction action = RobotAction.INTAKE;
    private CatzSuperstructure superstructure;
    private CatzOuttake outtake;
    private int level;
    private RobotContainer container;

    private int intakeIterationCounter = 0;

    private boolean actionAlreadyTaken = false;
    private boolean skipped = false;

    public DriveToCoralStation(PathPlannerPath newPath, RobotContainer container){
        super(newPath, container.getCatzDrivetrain(), false, container);
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
        intakeIterationCounter = 0;
        skipped = false;
        CatzSuperstructure.setChosenGamepiece(Gamepiece.CORAL);
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
            System.out.println("intaking!!!!!!");
            superstructure.setCurrentRobotAction(RobotAction.INTAKE, "intak");
            intakeIterationCounter++;
            if(!outtake.isDesiredCoralState(false) && intakeIterationCounter > 100) {
                skipped = true;
            }
        }

        // If we reached the target Destination
        if (super.isFinished()){
            // System.out.println("action: " + action.toString());
            if (container.getSelector().useFakeCoral){
                container.getSelector().hasCoralSIM = action == RobotAction.INTAKE;
            }
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
        return outtake.isDesiredCoralState(false);
    }
}
