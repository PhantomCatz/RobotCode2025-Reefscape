package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class JLKA extends AutoRoutineBase{
    public JLKA(){
        super("JLKA");

        AutoTrajectory Start2ToJ = getTrajectory("Start2 to J");
        AutoTrajectory JToTopLoad = getTrajectory("J to TopLoad");

        AutoTrajectory TopLoadToL = getTrajectory("TopLoad to L");
        AutoTrajectory LToTopLoad = getTrajectory("L to TopLoad");

        AutoTrajectory TopLoadToK = getTrajectory("TopLoad to K");
        AutoTrajectory KToTopLoad = getTrajectory("K to TopLoad");

        AutoTrajectory TopLoadToA = getTrajectory("TopLoad to A");

        prepRoutine(
            Start2ToJ,

            followTrajectoryAndScore(Start2ToJ, 4),
            followTrajectoryAndIntake(JToTopLoad),
            waitUntilCoralIntaked(),

            followTrajectoryAndScore(TopLoadToL, 4),
            followTrajectoryAndIntake(LToTopLoad),
            waitUntilCoralIntaked(),

            followTrajectoryAndScore(TopLoadToK, 4),
            followTrajectoryAndIntake(KToTopLoad),
            waitUntilCoralIntaked(),

            followTrajectoryAndScore(TopLoadToA, 4)
        );
    }
}
