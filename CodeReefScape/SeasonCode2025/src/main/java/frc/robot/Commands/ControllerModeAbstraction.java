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

    public static Command robotHandoff(RobotContainer container) {
        return Commands.either(
            AutomatedSequenceCmds.transferNoteToShooter(container), 
            AutomatedSequenceCmds.transferNoteToIntake(container), 
            ()->isModeSpeaker());
    }

    public static Command robotScore(RobotContainer container, Supplier<Boolean> override) {
        return Commands.either(
            AutomatedSequenceCmds.scoreSpeakerAutoAim(container, override), 
            AutomatedSequenceCmds.scoreAmp(container),
            ()->isModeSpeaker());
    }

    public static Command robotScoreSubwoofer(RobotContainer container, Supplier<Boolean> override) {
        return Commands.either(
            AutomatedSequenceCmds.scoreSpeakerSubwoofer(container, override), 
            AutomatedSequenceCmds.scoreAmp(container),
            ()->isModeSpeaker());
    }

    public static Command robotIntake(RobotContainer container) {
        return Commands.either(
            AutomatedSequenceCmds.noteDetectIntakeToShooter(container), 
            AutomatedSequenceCmds.noteDetectIntakeToAmpScoring(container),
            ()->isModeSpeaker());
    }

    public static Command cancelController(RobotContainer container) {
        return container.getCatzSuperstructure().cancelSuperStructureCommands();
    }


    public static void periodicDebug() {
    }
}
