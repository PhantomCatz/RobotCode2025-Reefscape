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

  public enum outtakeStates {
    ADJ_INIT,
    ADJ_BACK,
    SCORE,
    SCORE_L1,
    STOP,
    TEMP_RUN
  }

  private outtakeStates currentState = outtakeStates.STOP;

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

    switch (currentState) {
      case ADJ_INIT:
        case_adjustInit();
        break;
      case ADJ_BACK:
        case_adjustBack();
        break;
      case SCORE:
        case_shoot();
        break;
      case SCORE_L1:
        case_shootL1();
        break;
      case STOP: io.runVolts(0,0);
        break;
      case TEMP_RUN: io.runVolts(OUTTAKE_LT, OUTTAKE_RT);
    }
    // previousState = currentState;
  }

  // ============================================
  //
  //              Case methods
  //
  // ============================================

  private void case_adjustInit() {

    io.runVolts(INTAKE_SPD, INTAKE_SPD);

    if (inputs.bbreakFrntTriggered) {
      currentState = outtakeStates.ADJ_BACK;
    }
  }

  private void case_adjustBack() {

    io.runVolts(ADJ_SPD, ADJ_SPD);
    if (!inputs.bbreakBackTriggered) {
      currentState = outtakeStates.STOP;
    }
  }

  private void case_shoot() {

    io.runVolts(OUTTAKE_LT, OUTTAKE_RT);
    if(!inputs.bbreakFrntTriggered) {
        currentState = outtakeStates.STOP;
    }
  }
  private void case_shootL1() {

    io.runVolts(0.8, 0.05);
    if(!inputs.bbreakFrntTriggered) {
        currentState = outtakeStates.STOP;
    }
  }


  //=========================================================
  //
  //      Command access methods
  //
  //=========================================================


  public Command startIntaking() {
    return runOnce(() -> currentState = outtakeStates.ADJ_INIT).alongWith(Commands.print("hello"));
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

  public Command stopOuttake() {
    return runOnce(() -> io.runVolts(0,0));
  }
}
