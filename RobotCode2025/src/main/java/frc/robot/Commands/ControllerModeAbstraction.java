package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ControllerModeAbstraction {
    
    private static boolean m_isModeSpeaker;    
    public static boolean isModeSpeaker() {
        return m_isModeSpeaker;
    }

    public static Command sortModes(boolean isModeSpeaker) {
        return Commands.runOnce(()-> m_isModeSpeaker = isModeSpeaker);
    }

    // public static Command cancelController(RobotContainer container) {
    //     return container.getCatzSuperstructure().cancelSuperStructureCommands();
    // }


    public static void periodicDebug() {
    }
}
