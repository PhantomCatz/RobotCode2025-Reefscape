package frc.robot.Autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CatzConstants;

public class AutoRoutineBase {
    private AutoRoutine routine;

    public AutoRoutineBase(String name){
        routine = CatzConstants.autoFactory.newRoutine(name);
    }

    protected void prepRoutine(Command... sequence){
        routine.active().onTrue(
            Commands.sequence(sequence)
        );
    }

    protected AutoTrajectory getTrajectory(String name){
        return routine.trajectory(name);
    }

    protected AutoTrajectory getTrajectory(String name, int index){
        return routine.trajectory(name, index);
    }
}
