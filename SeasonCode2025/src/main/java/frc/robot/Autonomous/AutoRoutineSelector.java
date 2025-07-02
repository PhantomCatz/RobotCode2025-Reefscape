package frc.robot.Autonomous;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.routines.Test2Piece;

public class AutoRoutineSelector {
    public static final AutoRoutineSelector Instance = new AutoRoutineSelector();

    private AutoChooser autoSelector = new AutoChooser();

    private AutoRoutineSelector(){
        autoSelector.addRoutine("Test 2 Piece", () -> new Test2Piece().getRoutine());
    }

    public Command getSelectorCommand(){
        return autoSelector.selectedCommandScheduler();
    }

    public AutoChooser getAutoChooser(){
        return autoSelector;
    }
}
