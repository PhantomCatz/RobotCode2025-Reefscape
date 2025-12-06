package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class testinggg extends AutoRoutineBase{
    public testinggg(){
        super("JLKA");

        AutoTrajectory testinPath = getTrajectory("testinPath");

        prepRoutine(
            testinPath,
            followTrajectoryAndScore(testinPath, 4)
        );
    }
}
