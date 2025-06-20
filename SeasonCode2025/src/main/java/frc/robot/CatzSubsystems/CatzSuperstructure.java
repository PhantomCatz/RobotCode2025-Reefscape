//------------------------------------------------------------------------------------
// 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project. 
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems;

import java.util.Set;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TeleopPosSelector;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.ControllerLEDState;
import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

public class CatzSuperstructure extends VirtualSubsystem {
    private static CatzSuperstructure INSTANCE;

    public static CatzSuperstructure getInstance(){
        if(INSTANCE == null) INSTANCE = new CatzSuperstructure();
        return INSTANCE;
    }

    static @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/ChosenGamepiece")
    private Gamepiece chosenGamepiece = Gamepiece.CORAL;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/Level")
    private int level = 1;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/CurrentRobotState")
    private RobotState currentRobotState = RobotState.STOW;

    @Getter @AutoLogOutput(key = "CatzSuperstructure/CurrentRobotAction")
    private RobotAction currentRobotAction = RobotAction.STOW;

    @Getter @AutoLogOutput(key = "CatzSuperstructure/IsClimbEnabled")
    private static boolean isClimbEnabled = false;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/CurrentCoralState")
    private static CoralState currentCoralState = CoralState.NOT_IN_OUTTAKE;

    @Getter @AutoLogOutput(key = "CatzSuperstructure/robotactionCommand")
    private Command robotActionCommand = Commands.none();

    private Command previousAction = new InstantCommand();

    private final CatzLED LED = CatzLED.getInstance();

    public enum Gamepiece{
        CORAL,
        ALGAE
    }

    public enum CoralState{
        IN_OUTTAKE,
        NOT_IN_OUTTAKE,
        CORAL_ADJUSTING
    }

    public enum RobotState {
        STOW,
        INTAKE_CORAL_GROUND,
        INTAKE_CORAL_STATION,
        INTAKE_ALGAE_GROUND,
        GOBBLE_ALGAE,
        L1_CORAL,
        L2_CORAL,
        L3_CORAL,
        L4_CORAL,
        L1_AIMING,
        L2_AIMING,
        L3_AIMING,
        L4_AIMING,
        PROCESSOR,
        NET_ALGAE,
        BOT_ALGAE,
        TOP_ALGAE,
        CLIMBING,
        EXTENDING_CLIMB
    }

    public enum RobotAction {
        OUTTAKE,
        INTAKE,
        AIMING,
        INTAKE_GROUND,
        STOW,
        L4_AUTO_OUTTAKE
    }

    public enum LeftRight{
        LEFT(1),
        RIGHT(-1);

        public final int NUM;

        private LeftRight(int num){
          this.NUM  = num;
        }
    }

    private CatzSuperstructure() {
        this.level = 1;
    }

    public void setCurrentRobotAction(RobotAction action, String from){
        System.out.println("from: " + from + action);
        setCurrentRobotAction(action, level);
    }

    public void cycleGamePieceSelection() {
        if(chosenGamepiece == Gamepiece.CORAL) {
            chosenGamepiece = Gamepiece.ALGAE;
            System.out.println("Gamepiece: ALGAE");
        } else {
            chosenGamepiece = Gamepiece.CORAL;
            System.out.println("Gamepiece: CORAL");
        }
    }

    public void setClimbOverride(BooleanSupplier isClimbEnabled) {
        CatzSuperstructure.isClimbEnabled = isClimbEnabled.getAsBoolean();
        if (isClimbEnabled()) {
            LED.setControllerState(ControllerLEDState.CLIMB);
        }
        else {
            LED.setControllerState(ControllerLEDState.FULL_MANUAL);
        }
        System.out.println("CLimb Enabled" + isClimbEnabled);
    }

    public void setCurrentRobotAction(RobotAction action, int level) {
        robotActionCommand = Commands.print("No robot Action Selected");
        this.currentRobotAction = action;

        if(isClimbEnabled) {
            System.out.println("HI");
            LED.setControllerState(ControllerLEDState.CLIMB);
            return;
        }

        switch(currentRobotAction) {
            // Outtaking Algae or Coral
            case OUTTAKE:
                if(chosenGamepiece == Gamepiece.CORAL) {
                    System.out.println("Outtake Coral at L"+level);
                    switch (level) {
                        case 1:
                        currentRobotState = RobotState.L1_CORAL;
                        robotActionCommand = CatzStateCommands.L1Coral();
                        break;

                        case 2:
                        currentRobotState = RobotState.L2_CORAL;
                        robotActionCommand = CatzStateCommands.L2Coral();
                        break;

                        case 3:
                        currentRobotState = RobotState.L3_CORAL;
                        robotActionCommand = CatzStateCommands.L3Coral();
                        break;

                        case 4:
                        currentRobotState = RobotState.L4_CORAL;
                        robotActionCommand = CatzStateCommands.L4Coral();
                        break;
                    }
                } else {
                    switch(level) {
                        case 4:
                            currentRobotState = RobotState.PROCESSOR;
                            //System.out.println("Processor");
                            robotActionCommand = CatzStateCommands.processor();
                            break;
                        case 2:
                            currentRobotState = RobotState.NET_ALGAE;
                            //System.out.println("Net Algae");
                            robotActionCommand = CatzStateCommands.netAlgae();
                            break;
                        default:
                            robotActionCommand = Commands.none();
                            break;
                    }
                    // System.out.println("Processor");

                }
                break;

            case AIMING:

                if(chosenGamepiece == Gamepiece.CORAL) {
                    robotActionCommand = CatzStateCommands.LXElevator(level);
                    switch (level) {
                        case 1:
                        currentRobotState = RobotState.L1_AIMING;
                        break;

                        case 2:
                        currentRobotState = RobotState.L2_AIMING;
                        break;

                        case 3:
                        currentRobotState = RobotState.L3_AIMING;
                        break;

                        case 4:
                        currentRobotState = RobotState.L4_AIMING;
                        break;
                    }
                } else {
                    currentRobotState = RobotState.PROCESSOR;
                    robotActionCommand = CatzStateCommands.processor();
                    // System.out.println("Processor");

                }

                break;

            // Intake Algae From Reef or Coral from Coral Substation
            case INTAKE:
                if(chosenGamepiece == Gamepiece.CORAL) {
                    currentRobotState = RobotState.INTAKE_CORAL_STATION;
                    robotActionCommand = CatzStateCommands.intakeCoralStation();
                } else {
                    if(TeleopPosSelector.getInstance().recentNBAPos.getFirst() % 2 == 0) {
                        currentRobotState = RobotState.BOT_ALGAE;
                        System.out.println("BOT algae");
                        robotActionCommand = CatzStateCommands.botAlgae();
                    }else{
                        currentRobotState = RobotState.TOP_ALGAE;
                        System.out.println("TOP algae");
                        robotActionCommand = CatzStateCommands.topAlgae();
                    }

                }
                break;

            default:
            case STOW:
                // if(chosenGamepiece == Gamepiece.CORAL) {
                    currentRobotState = RobotState.STOW;
                    robotActionCommand = CatzStateCommands.stow();

                // } else {
                //     currentRobotState = RobotState.STOW;
                //     robotActionCommand = CatzStateCommands.algaeStow();
                // }
                break;

        }


        System.out.println("acton: " + currentRobotState);
        if(previousAction != robotActionCommand){
            robotActionCommand.schedule();
        }
        previousAction = robotActionCommand;

        Logger.recordOutput("CurrentRobotState/CurrentRobotState", currentRobotState.toString());
    }

    // public 

    // public Command L1Score(){
    //     return new 
    // }


    @Override
    public void periodic() {

        //----------------------------------------------------------------------------------
        // Logging
        //----------------------------------------------------------------------------------
        getChosenGamepiece();
        getLevel();
        getCurrentRobotState();
        getCurrentRobotAction();
        getCurrentCoralState();
        isClimbEnabled();

    }


}
