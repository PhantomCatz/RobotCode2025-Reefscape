// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

public class CatzSuperstructure extends VirtualSubsystem {

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/ChosenGamepiece")
    private Gamepiece chosenGamepiece = Gamepiece.CORAL;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/Level")
    private int level = 1;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/CurrentRobotState")
    private RobotState currentRobotState = RobotState.STOW;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/CurrentRobotAction")
    private RobotAction currentRobotAction = RobotAction.STOW;

    public enum Gamepiece{
        CORAL,
        ALGAE
    }

    public enum RobotState {
        STOW,
        INTAKE_CORAL_GROUND,
        INTAKE_CORAL_STATION,
        INTAKE_ALGAE,
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

    @Override
    public void periodic() {

        switch(currentRobotAction) {

            // Outtaking Algae or Coral
            case OUTTAKE:
                if(chosenGamepiece == Gamepiece.CORAL) {
                    switch (level) {
                        case 1:
                            currentRobotState = RobotState.L1_CORAL;
                        break;

                        case 2:
                        currentRobotState = RobotState.L2_CORAL;
                        break;

                        case 3:
                        currentRobotState = RobotState.L3_CORAL;
                        break;

                        case 4:
                        currentRobotState = RobotState.L4_CORAL;
                        break;
                    }
                } else {
                    currentRobotState = RobotState.PROCESSOR;
                    System.out.println("Processor");

                }
                break;

            // Intake Algae From Reef or Coral from Coral Substation
            case INTAKE:
                if(chosenGamepiece == Gamepiece.CORAL) {
                        currentRobotState = RobotState.INTAKE_CORAL_STATION;
                        System.out.println("Intake coral station");

                } else {
                    switch (level) {
                        case 2:
                            currentRobotState = RobotState.TOP_ALGAE;
                            System.out.println("TOP algae");

                            break;
                        case 4:
                            currentRobotState = RobotState.BOT_ALGAE;
                            System.out.println("BOT algae");

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

                } else {
                    currentRobotState = RobotState.INTAKE_ALGAE;
                    System.out.println("Intake algae");

                }
                break;

            // Sets All Mechanisms to Base Positions
            // default:
            case STOW:
                currentRobotState = RobotState.STOW;
                // System.out.println("STOW");

                break;

        }
    }
}
