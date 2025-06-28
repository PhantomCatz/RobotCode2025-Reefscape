package frc.robot.Autonomous;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutineSelector {
    public static final AutoRoutineSelector Instance = new AutoRoutineSelector();

    private AutoChooser autoSelector;

    private AutoRoutineSelector(){
        
    }

    public Command getSelectorCommand(){
        return autoSelector.selectedCommandScheduler();
    }

    public AutoChooser getAutoChooser(){
        return autoSelector;
    }
}
