// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzOuttake;
import static frc.robot.CatzSubsystems.CatzOuttake.OuttakeConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzSuperstructure.CoralState;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;

public class CatzOuttake extends SubsystemBase {

  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
  private RobotContainer container;
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
    RAMP_EJECT
  }

  @Setter @Getter
  private outtakeStates currentState = outtakeStates.STOP;
  private outtakeStates previousState = outtakeStates.STOP;

  public CatzOuttake(RobotContainer container) {
    this.container = container;
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


  /**
   *
   * @param isOuttaking This is the question asked. Is it out? Is it in?
   * @return
   */
  public boolean isDesiredCoralState(boolean isOuttaking){
    if (container.getSelector().useFakeCoral){
      return container.getSelector().hasCoralSIM;
    } else {
      if(isOuttaking){
        return (!inputs.bbreakBackTriggered) && (!inputs.bbreakFrntTriggered); //both beam breaks must be off to be out of the coral effector
      }else{
        return (inputs.bbreakBackTriggered || inputs.bbreakFrntTriggered); //only one of the beambreak must be triggered to be considered inside the coral effector
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RealInputs/Outtake", inputs);

    if(DriverStation.isDisabled()) {
      currentState = outtakeStates.STOP;
      io.runMotor(0.0, 0.0);
      io.runIntakesIntakeMotor(0.0);
    }

    if(currentState != previousState) {
      interationCounter = 0;
      intakeIterationCoutner = 0;
    }

    switch (currentState) {
      case ADJ_INIT:
      //Starts the intake until 0.1s has passed and if the BackBeambreak = Triggered, goes to ADJ_FWD. Else ADJ_BACK
        case_adjustInit();
        break;
      case ADJ_BACK:
      //Goes backwards until the BackBeambreak = triggered, then STOP
        case_adjustBack();
        break;
      case ADJ_FWD:
      //Goes forwards until the FrontBeambreak = triggered, then STOP
        case_adjustFwd();
        break;
      case SCORE:
      //Outtakes until the FrontBeambreak = triggered AND once 0.5s has passed
        case_shoot();
        break;
      case SCORE_L1:
      //Outtakes with speeds fit for L1 Scoring until FrontBeambreak = triggered AND once 1s has passed
        case_shootL1();
        break;
      case SCORE_L4:
      //Outtakes with speeds fit for L4 Scoring until FrontBeambreak = triggered AND once 0.5s has passed
        case_shootL4();
        break;
      case STOP:
        io.runIntakesIntakeMotor(0.0);
        io.runMotor(0,0);
        break;
      case RAMP_EJECT:
        io.runIntakesIntakeMotor(0.8);
        break;
    }

    Logger.recordOutput("Outtake/State", currentState);

    previousState = currentState;

  }

  // ============================================
  //
  //              Case methods
  //
  // ============================================

  private void case_adjustInit() {

    io.runMotor(INTAKE_SPD, INTAKE_SPD);
    io.runIntakesIntakeMotor(INTAKE_INTAKE_SPEED);

    CatzSuperstructure.setCurrentCoralState(CoralState.CORAL_ADJUSTING);

    if(inputs.bbreakFrntTriggered) {
      io.runMotor(0.0, 0.0);
      intakeIterationCoutner++;
      if(intakeIterationCoutner >= 20) {
        if(inputs.bbreakBackTriggered) {
          intakeIterationCoutner = 0;
          currentState = outtakeStates.ADJ_FWD;
        } else {
          intakeIterationCoutner = 0;
          currentState = outtakeStates.ADJ_BACK;
        }
      }
    }
  }

  private void case_adjustBack() {
    io.runMotor(-ADJ_SPD, -ADJ_SPD);

    CatzSuperstructure.setCurrentCoralState(CoralState.NOT_IN_OUTTAKE);

    if (inputs.bbreakBackTriggered) {
      currentState = outtakeStates.STOP;
      CatzSuperstructure.setCurrentCoralState(CoralState.IN_OUTTAKE);
    }
  }

  private void case_adjustFwd() {
    io.runMotor(ADJ_SPD, ADJ_SPD);
    if (!inputs.bbreakBackTriggered) {
      currentState = outtakeStates.ADJ_BACK;
    }
  }


  private void case_shoot() {
    io.runIntakesIntakeMotor(0.0);
    io.runMotor(OUTTAKE_LT, OUTTAKE_RT);
    interationCounter++;
    if(!inputs.bbreakFrntTriggered && interationCounter >= 25) { //0.02s per iteration
        interationCounter = 0;
        CatzSuperstructure.setCurrentCoralState(CoralState.NOT_IN_OUTTAKE);
        currentState = outtakeStates.STOP;
    }
  }

  private void case_shootL1() {
    io.runIntakesIntakeMotor(0.0);
    io.runMotor(OUTTAKE_L1_LT, OUTTAKE_L1_RT);
    interationCounter++;
    if(!inputs.bbreakFrntTriggered&& interationCounter >= 50) {
      interationCounter = 0;
      CatzSuperstructure.setCurrentCoralState(CoralState.NOT_IN_OUTTAKE);
      currentState = outtakeStates.STOP;
    }
  }

  private void case_shootL4() {
    io.runMotor(OUTTAKE_L4, OUTTAKE_L4);
    interationCounter++;
    if(!inputs.bbreakFrntTriggered && interationCounter >= 40) {
        interationCounter = 0;
        CatzSuperstructure.setCurrentCoralState(CoralState.NOT_IN_OUTTAKE);
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


  public Command rampEject() {
    return runOnce(() -> currentState = outtakeStates.RAMP_EJECT);
  }

  public Command startOuttake() {
    return runOnce(() -> currentState = outtakeStates.SCORE);
  }

  public Command outtakeL1() {
    return runOnce(() -> currentState = outtakeStates.SCORE_L1);
  }

  public Command outtakeL4() {

    return runOnce(() -> {currentState = outtakeStates.SCORE_L4;interationCounter = 0;});
  }

  public Command stopOuttake() {
    return runOnce(() -> currentState = outtakeStates.STOP);
  }
}
