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
package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;



public class CatzElevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double elevatorSpeed = 0.0;
  private double elevatorFeedForward = 0.0;
  private int settlingCounter = 0;
  private boolean breakModeEnabled = true;

  private ElevatorPosition targetPosition = ElevatorPosition.PosStow;
  private ElevatorPosition prevTargetPositon = ElevatorPosition.PosNull;
  private ElevatorPosition previousLoggedPosition = ElevatorPosition.PosNull;

  private boolean isElevatorInPos = false;

  @RequiredArgsConstructor
  public static enum ElevatorPosition {
      //TO CHANGE HEIGHT GO TO ElevatorConstants.java
      PosLimitSwitchStow(() -> 0.0),
      PosStow(() -> STOW_HEIGHT),
      PosCoastStow(() -> COAST_STOW_HEIGHT),
      PosL1(() -> L1_HEIGHT),
      PosL2(() -> L2_HEIGHT),
      PosL3(() -> L3_HEIGHT),
      PosL4(() -> L4_HEIGHT),
      PosL4Adj(() -> L4_CORAL_ADJ),
      AlgaeBotTransition(() -> ALGAE_BOT),
      PosBotBot(() -> BOT_BOT_ALGAE),
      PosBotTop(() -> BOT_TOP_ALGAE),
      PosManual(new LoggedTunableNumber("Elevator/ScoreSourceSetpoint",0.0)),
      PosNull(() -> -1.0);

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getTargetPositionInch() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  public CatzElevator() {
    if(isElevatorDisabled) {
      io = new ElevatorIONull();
      System.out.println("Elevator Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new ElevatorIOReal();
          System.out.println("Elevator Configured for Real");
        break;
        case REPLAY:
          io = new ElevatorIOReal() {};
          System.out.println("Elevator Configured for Replayed simulation");
        break;
        default:
          io = new ElevatorIONull();
          System.out.println("Elevator Unconfigured");
        break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RealInputs/Elevator", inputs);

    isElevatorInPos = isElevatorInPosition();

    //--------------------------------------------------------------------------------------------------------
    // Update controllers when user specifies
    //--------------------------------------------------------------------------------------------------------
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGainsSlot0(slot0_kP.get(),
                               slot0_kI.get(),
                               slot0_kD.get(),
                               slot0_kS.get(),
                               slot0_kV.get(),
                               slot0_kA.get(),
                               slot0_kG.get()
        ),
        slot0_kP,
        slot0_kI,
        slot0_kD,
        slot0_kS,
        slot0_kV,
        slot0_kA,
        slot0_kG
    );

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGainsSlot1(slot1_kP.get(),
                               slot1_kI.get(),
                               slot1_kD.get(),
                               slot1_kS.get(),
                               slot1_kV.get(),
                               slot1_kA.get(),
                               slot1_kG.get()
        ),
        slot1_kP,
        slot1_kI,
        slot1_kD,
        slot1_kS,
        slot1_kV,
        slot1_kA,
        slot1_kG
    );

    LoggedTunableNumber.ifChanged(hashCode(),
        ()-> io.setMotionMagicParameters(mmCruiseVelocity.get(), mmAcceleration.get(), mmJerk.get()),
        mmCruiseVelocity,
        mmAcceleration,
        mmJerk);

    if(previousLoggedPosition != targetPosition) {
      prevTargetPositon = targetPosition;
    }

    //---------------------------------------------------------------------------------------------------------------------------
    //    Limit switch position setting
    //---------------------------------------------------------------------------------------------------------------------------
    if(inputs.isBotLimitSwitched) {
      //io.resetPosition(ElevatorPosition.PosLimitSwitchStow.getTargetPositionRads());
    }

    //---------------------------------------------------------------------------------------------------------------------------

    //    Control Mode setting
    //---------------------------------------------------------------------------------------------------------------------------
    if(DriverStation.isDisabled()) {
      // Disabled
      io.stop();
      targetPosition = ElevatorPosition.PosNull;
    } else if(targetPosition != ElevatorPosition.PosNull &&
              targetPosition != ElevatorPosition.PosManual){
      // if(targetPosition == ElevatorPosition.PosCoastStow) {
      //   //hopefully a slow descent from algae intake positions to stow w an algae; regular PID effected desecent is too extreme for algae pivot arm
      //   io.stop();
      // Setpoint PID
      if(targetPosition == ElevatorPosition.PosStow) {
        // Safety Stow

        if(getElevatorPositionInch() < 2.0) {
          io.stop();
        } else {
          io.runSetpointDown(targetPosition.getTargetPositionInch());
        }
      } else {
        //Setpoint PID
        io.runSetpointUp(targetPosition.getTargetPositionInch());
      }
    } else if (targetPosition == ElevatorPosition.PosManual) {
      io.runMotor(elevatorSpeed);

      Logger.recordOutput("Elevator/Manual Speed", elevatorSpeed*10);
      Logger.recordOutput("Elevator/Manual RPS", inputs.velocityInchPerSec/ElevatorConstants.FINAL_RATIO);
    } else {
      // Nothing happening
      io.stop();
    }

    //----------------------------------------------------------------------------------------------------------------------------
    // Logging
    //----------------------------------------------------------------------------------------------------------------------------


    Logger.recordOutput("Elevator/CurrentRadians", getElevatorPositionInch());
    Logger.recordOutput("Elevator/prevtargetPosition", prevTargetPositon.getTargetPositionInch());
    Logger.recordOutput("Elevator/logged prev targetPosition", previousLoggedPosition.getTargetPositionInch());
    Logger.recordOutput("Elevator/isElevatorInPos", isElevatorInPosition());
    Logger.recordOutput("Elevator/targetPosition", targetPosition.getTargetPositionInch());

    // Target Postioin Logging
    previousLoggedPosition = targetPosition;
  }

  //--------------------------------------------------------------------------------------------------------------------------
  //
  //  Elevator Setpos Commands
  //
  //--------------------------------------------------------------------------------------------------------------------------
  public Command Elevator_LX(int level){
    switch (level){
      case 1:
        return Elevator_L1();

      case 2:
        return Elevator_L2();

      case 3:
        return Elevator_L3();

      case 4:
        return Elevator_L4();

      default:
        System.out.println("Invalid elevator level!");
        return new InstantCommand();
    }
  }

  public boolean isElevatorInPos(){
    return isElevatorInPos;
  }

  public Command Elevator_Stow() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosStow));
  }

  public Command Elevator_Coast_Stow() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosCoastStow));
  }

  public Command Elevator_L1() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL1));
  }

  public Command Elevator_L2() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL2));
  }

  public Command Elevator_L3() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL3));
  }

  public Command Elevator_L4() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL4));
  }

  public Command Elevator_L4_Adj() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL4Adj));
  }

  public Command Elevator_Bot_Transition() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.AlgaeBotTransition));
  }
  public Command Elevator_BOT_BOT() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosBotBot));
  }

  public Command Elevator_BOT_TOP() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosBotTop));
  }

  public void setElevatorPos(ElevatorPosition target) {
    System.out.println("set elevator pose to: " + target);
    settlingCounter = 0;
    this.targetPosition = target;
  }

  //--------------------------------------------------------------------------
  //
  //        Elevator Motor methods
  //
  //--------------------------------------------------------------------------

  public double getElevatorPositionInch() {
    return inputs.positionInch;
  }

  private boolean isElevatorInPosition() {
    boolean isElevatorSettled = false;
    boolean isElevatorInPos = (Math.abs((getElevatorPositionInch() - targetPosition.getTargetPositionInch())) < 1.0);
    if(isElevatorInPos) {
      settlingCounter++;
      if(settlingCounter >= 8) {
         isElevatorSettled = true;
        // settlingCounter = 0;
         //System.out.println("////////////ELEVATOR SETTLED FOR .2 SECONDS///////////////////");
      }
    } else {
      isElevatorSettled = false;
      settlingCounter = 0;
    }
    return isElevatorSettled;
  }

  public void setBrakeMode(boolean enabled) {
    if(breakModeEnabled == false) return;
    breakModeEnabled = true;
    io.setBrakeMode(breakModeEnabled);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityInchPerSec;
  }

  public void elevatorFullManual(double manualPower) {
    this.elevatorSpeed = manualPower;
    targetPosition = ElevatorPosition.PosManual;
  }

  public Command elevatorFullManual(Supplier<Double> manuaSupplier) {

    return run(() -> elevatorFullManual(manuaSupplier.get()));
  }
}
