// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.TeleopPosSelector;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;

public class MoveScore extends Command{
    private final RobotContainer container;
    private final int level;

    private Command driveCommand;
    private CatzSuperstructure superstructure;


    public MoveScore(RobotContainer container, int level){
        addRequirements(container.getCatzDrivetrain(), container.getCatzOuttake(), container.getCatzElevator());

        this.container = container;
        this.level = level;
    }

    @Override
    public void initialize(){
        TeleopPosSelector selector = container.getSelector();
        superstructure = container.getSuperstructure();

        Translation2d startPose = CatzRobotTracker.getInstance().getEstimatedPose().getTranslation();
        Pose2d goalPose = selector.calculateReefPose(selector.getClosestReefPos(startPose), false, false);
        PathPlannerPath scorePath = selector.getPathfindingPath(startPose, goalPose);

        driveCommand = new TrajectoryDriveCmd(scorePath, container.getCatzDrivetrain(), true, container);
        driveCommand.initialize();

        superstructure.setCurrentRobotAction(RobotAction.AIMING, level);
    }

    @Override
    public void execute(){
        if(!driveCommand.isFinished()){
            driveCommand.execute();
        }else{
            superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, level);
        }
    }

    @Override
    public boolean isFinished(){
        return container.getCatzOuttake().isDesiredCoralState(true);
    }

    @Override
    public void end(boolean interrupted){
        driveCommand.end(interrupted);
    }
}
