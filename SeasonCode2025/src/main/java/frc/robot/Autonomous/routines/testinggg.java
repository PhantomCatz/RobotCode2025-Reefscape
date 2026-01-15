package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import frc.robot.Autonomous.AutoRoutineBase;

public class testinggg extends AutoRoutineBase{
    public testinggg(){
        super("testinggg");

        // AutoTrajectory Start3ToI = getTrajectory("Start3 to I");
        // AutoTrajectory testinPath = getTrajectory("testinPath");
        AutoTrajectory speed = getTrajectory("kindafast");
        AutoTrajectory small = getTrajectory("small path");

        prepRoutine(
            speed,
            followTrajectoryWithAccuracy(speed),
            // followTrajectoryWithAccuracy(testinPath).until(()-> Detection.Instance.hasCoral()).andThen(TeleopPosSelector.Instance.autoIntakeMode())
            followTrajectoryWithAccuracy(small)//.until(()-> Detection.Instance.hasCoral()).andThen(TeleopPosSelector.Instance.autoIntakeMode())

        );
    }
}
