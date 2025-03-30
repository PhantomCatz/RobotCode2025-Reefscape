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

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

public class CatzSuperstructure extends VirtualSubsystem {

    private RobotContainer container;

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
    private static CoralState currentCoralState = CoralState.IN_OUTTAKE;

    @Getter @AutoLogOutput(key = "CatzSuperstructure/robotactionCommand")
    private Command robotActionCommand = Commands.none();

    private Command previousAction = new InstantCommand();

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

    public CatzSuperstructure(RobotContainer container) {
        this.container = container;
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
        System.out.println("CLimb Enabled" + isClimbEnabled);
    }

    public void setCurrentRobotAction(RobotAction action, int level) {
        robotActionCommand = Commands.print("No robot Action Selected");
        this.currentRobotAction = action;

        if(isClimbEnabled) {
            System.out.println("HI");
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
                        robotActionCommand = CatzStateCommands.L1Coral(container);
                        break;

                        case 2:
                        currentRobotState = RobotState.L2_CORAL;
                        robotActionCommand = CatzStateCommands.L2Coral(container);
                        break;

                        case 3:
                        currentRobotState = RobotState.L3_CORAL;
                        robotActionCommand = CatzStateCommands.L3Coral(container);
                        break;

                        case 4:
                        currentRobotState = RobotState.L4_CORAL;
                        robotActionCommand = CatzStateCommands.L4Coral(container);
                        break;
                    }
                } else {
                    switch(level) {
                        case 4:
                            currentRobotState = RobotState.PROCESSOR;
                            //System.out.println("Processor");
                            robotActionCommand = CatzStateCommands.processor(container);
                            break;
                        case 2:
                            currentRobotState = RobotState.NET_ALGAE;
                            //System.out.println("Net Algae");
                            robotActionCommand = CatzStateCommands.netAlgae(container);
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
                    robotActionCommand = CatzStateCommands.LXElevator(container, level);
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
                    robotActionCommand = CatzStateCommands.processor(container);
                    // System.out.println("Processor");

                }

                break;

            // Intake Algae From Reef or Coral from Coral Substation
            case INTAKE:
                if(chosenGamepiece == Gamepiece.CORAL) {
                    currentRobotState = RobotState.INTAKE_CORAL_STATION;
                    robotActionCommand = CatzStateCommands.intakeCoralStation(container);
                } else {
                switch (level) {
                    case 2:
                        currentRobotState = RobotState.TOP_ALGAE;
                        System.out.println("TOP algae");
                        robotActionCommand = CatzStateCommands.botAlgae(container);
                        break;
                    case 4:
                        currentRobotState = RobotState.BOT_ALGAE;
                        System.out.println("BOT algae");
                        robotActionCommand = CatzStateCommands.topAlgae(container);
                        break;
                    default:
                        break;
                }
            }
                break;

            // Intake Algae Ground or Coral Ground

            case INTAKE_GROUND:
                if(chosenGamepiece == Gamepiece.CORAL) {
                    currentRobotState = RobotState.INTAKE_CORAL_GROUND;
                    System.out.println("Intake coral ground");
                    robotActionCommand = CatzStateCommands.intakeCoralGround(container);


                } else {
                    currentRobotState = RobotState.GOBBLE_ALGAE;
                    System.out.println("Gob algae");
                    robotActionCommand = container.getAlgaePivot().AlgaePivot_Punch();

                }
            break;

            default:
            case STOW:
                currentRobotState = RobotState.STOW;
                robotActionCommand = CatzStateCommands.stow(container);
                break;

        }


        System.out.println("acton: " + currentRobotState);
        if(previousAction != robotActionCommand){
            robotActionCommand.schedule();
        }
        previousAction = robotActionCommand;

        Logger.recordOutput("CurrentRobotState/CurrentRobotState", currentRobotState);

    }


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
