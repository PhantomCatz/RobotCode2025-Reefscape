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
    ADJ_FINAL,
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
      case ADJ_FINAL:
        case_adjustFinal();
        break;
      case SCORE:
        case_shoot();
        break;
        // case STOP: runMotor(1.0,1.0);
        // break;
    }
    // previousState = currentState;
  }

  public Command runMotor() {
    return run(() -> io.runMotor(outtakeLeft, outtakeRight));
  }

  public Command runMotorBck(double spd) {
    return run(() -> io.runMotorBck(spd));
  }

  public Command runMotorLeft(double spd) {
    return run(() -> runMotorLeft(spd));
  }

  public Command runMotorRight(double spd) {
    return run(() -> runMotorRight(spd));
  }

  public Command startIntaking() {
    return run(() -> currentState = outtakeStates.ADJ_INIT);
  }

  public Command startOuttake() {
    return run(() -> currentState = outtakeStates.SCORE);
  }

  // ============================================
  //
  //              Case methods
  //
  // ============================================

  private void case_adjustInit() {
    // runMotor(adj_speed);

    if (inputs.bbreakFrntTriggered) {
      currentState = outtakeStates.ADJ_BACK;
    }
  }

  private void case_adjustBack() {
    if (!inputs.bbreakBackTriggered) {
      runMotorBck(adj_speed);
      currentState = outtakeStates.ADJ_FINAL;
    }
  }

  private void case_adjustFinal() {
    if (inputs.bbreakBackTriggered) {
      currentState = outtakeStates.STOP;
    }
  }

  private void case_shoot() {
    runMotorLeft(outtakeLeft);
    runMotorRight(outtakeRight);
    // if(!inputs.bbreakFrntTriggered) {
    //     currentState = outtakeStates.STOP;
    // }

  }
}
