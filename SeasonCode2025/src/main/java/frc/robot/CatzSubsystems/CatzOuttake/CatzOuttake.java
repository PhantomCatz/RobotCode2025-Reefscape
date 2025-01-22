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
    STOP
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

  double outtakeRight = 0.5;
  double outtakeLeft = 0.5;
  double adj_speed = 0.2;

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
    io.runMotor(adj_speed, adj_speed);
    if (inputs.bbreakFrntTriggered) {
      currentState = outtakeStates.ADJ_BACK;
    }
  }

  private void case_adjustBack() {
    System.out.println("using case_adjustBack");

    io.runMotor(-adj_speed, -adj_speed);
    if (inputs.bbreakBackTriggered) {
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
    return run(() -> currentState = outtakeStates.ADJ_INIT);
  }

  public Command startOuttake() {
    return run(() -> currentState = outtakeStates.SCORE);
  }

}
