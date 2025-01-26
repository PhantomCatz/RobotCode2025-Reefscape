// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzOuttake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import org.littletonrobotics.junction.Logger;

public class CatzOuttake extends SubsystemBase {

  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  // private static LoggedTunableNumber tunableNumber = new LoggedTunableNumber("Intake/MotorPower",
  // 0.5);

  public enum outtakeStates {
    ADJ_INIT,
    ADJ_BACK,
    SCORE,
    STOP,
    TEMP_RUN
  }

  private outtakeStates currentState = outtakeStates.STOP;

  // private outtakeStates previousState = outtakeStates.STOP;

  public CatzOuttake() {
    switch (CatzConstants.hardwareMode) {
      case REAL:
        io = new OuttakeIOSparkmax();
        break;
      case REPLAY:
        io = new OuttakeIOSparkmax() {};
        break;
      default:
        io = new OuttakeIOSparkmax();
        break;
    }
  }

  double outtakeRight = 0.3;
  double outtakeLeft = 0.3;
  double intakeSpeed = 0.3;
  double adjustSpeed = 0.1;

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("intake", inputs);

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
      case STOP: io.runMotor(0,0);
        break;
      case TEMP_RUN: io.runMotor(outtakeLeft, outtakeRight);
    }
    // previousState = currentState;
  }

  // ============================================
  //
  //              Case methods
  //
  // ============================================

  private void case_adjustInit() {
    System.out.println("using case_adjustInit");

    io.runMotor(intakeSpeed, intakeSpeed);

    if (inputs.bbreakFrntTriggered) {
      currentState = outtakeStates.ADJ_BACK;
    }
  }

  private void case_adjustBack() {
    System.out.println("using adjus");

    io.runMotor(adjustSpeed, adjustSpeed);
    if (!inputs.bbreakBackTriggered) {
      currentState = outtakeStates.STOP;
    }
  }

  private void case_shoot() {
    System.out.println("using case_shoot");

    io.runMotor(outtakeLeft, outtakeRight);
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
    return runOnce(() -> currentState = outtakeStates.ADJ_INIT);
  }

  public Command tempIntake() {
    return runOnce(() -> currentState = outtakeStates.TEMP_RUN);
  }

  public Command startOuttake() {
    return runOnce(() -> currentState = outtakeStates.SCORE);
  }

}
