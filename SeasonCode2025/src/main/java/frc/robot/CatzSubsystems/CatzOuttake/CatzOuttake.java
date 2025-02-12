// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzOuttake;
import static frc.robot.CatzSubsystems.CatzOuttake.OuttakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;

import org.littletonrobotics.junction.Logger;

public class CatzOuttake extends SubsystemBase {

  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private int interationCounter = 0;
  private int intakeIterationCoutner = 0;

  public enum outtakeStates {
    ADJ_INIT,
    ADJ_BACK,
    ADJ_FWD,
    SCORE,
    SCORE_L1,
    SCORE_L4,
    STOP,
    TEMP_RUN
  }

  private outtakeStates currentState = outtakeStates.STOP;
  private outtakeStates previousState = outtakeStates.STOP;

  public CatzOuttake() {
    if(isOuttakeDisabled) {
      io = new OuttakeIONull();
      System.out.println("Outtake Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new OuttakeIOReal();
          System.out.println("Outtake Configured for Real");
        break;
        case REPLAY:
          io = new OuttakeIOReal() {};
          System.out.println("Outtake Configured for Replayed simulation");
        break;
        default:
          io = new OuttakeIONull();
          System.out.println("Outtake Unconfigured");
        break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/Outtake", inputs);

    if(currentState != previousState) {
      interationCounter = 0;
      intakeIterationCoutner = 0;
    }

    switch (currentState) {
      case ADJ_INIT:
        case_adjustInit();
        break;
      case ADJ_BACK:
        case_adjustBack();
        break;
      case ADJ_FWD:
        case_adjustFwd();
        break;
      case SCORE:
        case_shoot();
        break;
      case SCORE_L1:
        case_shootL1();
        break;
      case SCORE_L4:
        case_shootL4();
        break;
      case STOP:
        io.runMotor(0,0);
        break;
      case TEMP_RUN:
        io.runMotor(OUTTAKE_LT, OUTTAKE_RT);
        break;
    }

    previousState = currentState;

  }

  // ============================================
  //
  //              Case methods
  //
  // ============================================

  private void case_adjustInit() {

    io.runMotor(INTAKE_SPD, INTAKE_SPD);
    if(inputs.bbreakFrntTriggered) {
      io.runMotor(0.0, 0.0);
      intakeIterationCoutner++;
      if(intakeIterationCoutner >= 5) {
        if(inputs.bbreakBackTriggered) {
          intakeIterationCoutner = 0;
          System.out.println("going to adj_fwd");
          currentState = outtakeStates.ADJ_FWD;
        } else {
          intakeIterationCoutner = 0;
          System.out.println("going to adj_bck");
          currentState = outtakeStates.ADJ_BACK;
        }
      }
    }
  }

  private void case_adjustBack() {
    io.runMotor(-ADJ_SPD, -ADJ_SPD);
    if (inputs.bbreakBackTriggered) {
      currentState = outtakeStates.STOP;
      System.out.println("stopping adjbck");

    }
  }

  private void case_adjustFwd() {
    io.runMotor(ADJ_SPD, ADJ_SPD);
    if (!inputs.bbreakBackTriggered) {
      currentState = outtakeStates.STOP;
      System.out.println("stopping adjfwd");
    }
  }


  private void case_shoot() {
    io.runMotor(OUTTAKE_LT, OUTTAKE_RT);
    interationCounter++;
    if(!inputs.bbreakFrntTriggered && interationCounter >= 25) {
        interationCounter = 0;
        currentState = outtakeStates.STOP;
    }
  }
  private void case_shootL1() {
    io.runMotor(OUTTAKE_L1_LT, OUTTAKE_L1_RT);
    interationCounter++;
    if(!inputs.bbreakFrntTriggered&& interationCounter >= 100) {
      interationCounter = 0;
      currentState = outtakeStates.STOP;
    }
  }

  private void case_shootL4() {
    io.runMotor(OUTTAKE_L4, OUTTAKE_L4);
    interationCounter++;
    if(!inputs.bbreakFrntTriggered && interationCounter >= 25) {
        interationCounter = 0;
        currentState = outtakeStates.STOP;
    }
  }


  //=========================================================
  //
  //      Command access methods
  //
  //=========================================================


  public Command startIntaking() {
    return runOnce(() -> currentState = outtakeStates.ADJ_INIT).alongWith(Commands.print("Commanded: startIntaking"));
  }


  public Command tempIntake() {
    return runOnce(() -> currentState = outtakeStates.TEMP_RUN);
  }

  public Command startOuttake() {
    return runOnce(() -> currentState = outtakeStates.SCORE);
  }

  public Command outtakeL1() {
    return runOnce(() -> currentState = outtakeStates.SCORE_L1);
  }

  public Command outtakeL4() {
    return runOnce(() -> currentState = outtakeStates.SCORE_L4);
  }

  public Command stopOuttake() {
    return runOnce(() -> io.runMotor(0,0));
  }
}
