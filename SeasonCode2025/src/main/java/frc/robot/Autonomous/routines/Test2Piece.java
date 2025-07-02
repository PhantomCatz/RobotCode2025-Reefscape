package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class Test2Piece extends AutoRoutineBase{
    public Test2Piece(){
        super("Test 2 Piece");

        AutoTrajectory start3ToI = getTrajectory("Start3 to I");
        AutoTrajectory ItoTopLoad = getTrajectory("I to TopLoad");
        AutoTrajectory TopLoadToJ = getTrajectory("TopLoad to J");

        prepRoutine(
            followTrajectoryAndScore(start3ToI, 4),
            followTrajectoryAndIntake(ItoTopLoad),
            waitUntilCoralIntaked(),
            followTrajectoryAndScore(TopLoadToJ, 4)
        );
    }
}
