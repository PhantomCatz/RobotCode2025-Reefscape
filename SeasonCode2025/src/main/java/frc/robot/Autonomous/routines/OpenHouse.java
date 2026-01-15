package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class OpenHouse extends AutoRoutineBase{
    public OpenHouse(){
        super("Open House");

        AutoTrajectory uno = getTrajectory("Test Path");

        prepRoutine(uno,
            followTrajectoryAndScore(uno, 4)
        );
    }
}
