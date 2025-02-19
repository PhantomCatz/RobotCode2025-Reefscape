// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

public class CatzSuperstructure extends VirtualSubsystem {

    private RobotContainer container;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/ChosenGamepiece")
    private Gamepiece chosenGamepiece = Gamepiece.CORAL;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/Level")
    private int level = 1;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/CurrentRobotState")
    private RobotState currentRobotState = RobotState.STOW;

    @Getter @AutoLogOutput(key = "CatzSuperstructure/CurrentRobotAction")
    private RobotAction currentRobotAction = RobotAction.STOW;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/CurrentCoralState")
    private static CoralState currentCoralState = CoralState.NOT_IN_OUTTAKE;

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
        L1_CORAL,
        L2_CORAL,
        L3_CORAL,
        L4_CORAL,
        PROCESSOR,
        BOT_ALGAE,
        TOP_ALGAE,
        CLIMB
    }

    public enum RobotAction {
        OUTTAKE,
        INTAKE,
        INTAKE_GROUND,
        STOW
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

    public void setCurrentRobotAction(RobotAction action){
        setCurrentRobotAction(action, level);
    }

    public void setCurrentRobotAction(RobotAction action, int level) {
        Command robotActionCommand = Commands.print("No robot Action Selected");
        this.currentRobotAction = action;
        switch(currentRobotAction) {

            // Outtaking Algae or Coral
            case OUTTAKE:
                container.getCatzOuttake().setCoral(false);
                System.out.println("ehehahaha");

                if(chosenGamepiece == Gamepiece.CORAL) {
                    switch (level) {
                        case 1:
                        currentRobotState = RobotState.L1_CORAL;
                        robotActionCommand = CatzStateCommands.L1Coral(container);
                        break;

                        case 2:
                        currentRobotState = RobotState.L2_CORAL;
                        robotActionCommand =  CatzStateCommands.L2Coral(container);
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
                    currentRobotState = RobotState.PROCESSOR;
                    robotActionCommand = CatzStateCommands.processor(container);
                    System.out.println("Processor");

                }
                break;

            // Intake Algae From Reef or Coral from Coral Substation
            case INTAKE:
                container.getCatzOuttake().setCoral(true);

                if(chosenGamepiece == Gamepiece.CORAL) {
                        currentRobotState = RobotState.INTAKE_CORAL_STATION;
                        System.out.println("Intake coral station");
                        robotActionCommand = CatzStateCommands.intakeCoralStation(container);
                } else {
                    switch (level) {
                        case 2:
                            currentRobotState = RobotState.TOP_ALGAE;
                            System.out.println("TOP algae");
                            robotActionCommand = CatzStateCommands.topAlgae(container);
                            break;
                        case 4:
                            currentRobotState = RobotState.BOT_ALGAE;
                            System.out.println("BOT algae");
                            robotActionCommand = CatzStateCommands.botAlgae(container);
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
                    currentRobotState = RobotState.INTAKE_ALGAE_GROUND;
                    System.out.println("Intake algae");
                    robotActionCommand = CatzStateCommands.intakeAlgae(container);

                }
                break;

            // Sets All Mechanisms to Base Positions
            default:
            case STOW:
                currentRobotState = RobotState.STOW;
                robotActionCommand = CatzStateCommands.stow(container);
                break;

        }

        robotActionCommand.schedule();


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

    }


}
