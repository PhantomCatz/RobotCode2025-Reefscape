// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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

  static LoggedTunableNumber tunableNumber = new LoggedTunableNumber("Elevator/MotorPower", 0.1);
  static LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.17);
  static LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
  static LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0006);

  static LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0);
  static LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0);
  static LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0);

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
      PosL1(() -> L1_HEIGHT),
      PosL2(() -> L2_HEIGHT),
      PosL3(() -> L3_HEIGHT),
      PosL4(() -> L4_HEIGHT),
      PosL4Adj(() -> L4_CORAL_ADJ),
      PosBotBot(() -> BOT_BOT_ALGAE),
      PosBotTop(() -> BOT_TOP_ALGAE),
      PosManual(new LoggedTunableNumber("Elevator/ScoreSourceSetpoint",0.0)),
      PosNull(() -> -1.0);

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getTargetPositionRads() {
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
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
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
      io.setPosition(ElevatorPosition.PosLimitSwitchStow.getTargetPositionRads());
    }

    //---------------------------------------------------------------------------------------------------------------------------
    //    Feed Foward
    //---------------------------------------------------------------------------------------------------------------------------

    // double additionalGain = 0.0;
    // switch (targetPosition) {
    //   case  PosL1:
    //     additionalGain = 0.00;
    //     break;
    //   case  PosL2:
    //     additionalGain = 0.025;
    //     break;
    //   case  PosL3:
    //     additionalGain = 0.0;
    //     break;
    //   case  PosL4:
    //     additionalGain = -0.2;
    //     break;
    //   default:
    //     break;
    // }
    elevatorFeedForward = gains.kG();// + additionalGain;

    //---------------------------------------------------------------------------------------------------------------------------
    //    Control Mode setting
    //---------------------------------------------------------------------------------------------------------------------------
    if(DriverStation.isDisabled()) {
      // Disabled
      io.stop();
      targetPosition = ElevatorPosition.PosNull;
    } else if(targetPosition != ElevatorPosition.PosNull &&
              targetPosition != ElevatorPosition.PosManual){
      // Setpoint PID
      if(targetPosition == ElevatorPosition.PosStow) {
        // Safety Stow
        if(getElevatorPositionRads() < 9.50) {
          io.stop();
        } else {
          io.runSetpoint(targetPosition.getTargetPositionRads(), elevatorFeedForward);
        }
      } else {
        //Setpoint PID
        io.runSetpoint(targetPosition.getTargetPositionRads(), elevatorFeedForward);
      }
    } else if (targetPosition == ElevatorPosition.PosManual) {
      io.runMotor(elevatorSpeed);
      // System.out.println("Running Elevator Motor");
    } else {
      // Nothing happening
      // System.out.println("Stopping running motor");
      io.stop();
    }

    //----------------------------------------------------------------------------------------------------------------------------
    // Logging
    //----------------------------------------------------------------------------------------------------------------------------
    Logger.recordOutput("Elevator/CurrentRadians", getElevatorPositionRads());
    Logger.recordOutput("Elevator/prevtargetPosition", prevTargetPositon.getTargetPositionRads());
    Logger.recordOutput("Elevator/logged prev targetPosition", previousLoggedPosition.getTargetPositionRads());
    Logger.recordOutput("Elevator/isElevatorInPos", isElevatorInPos);
    Logger.recordOutput("Elevator/targetPosition", targetPosition.getTargetPositionRads());

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

  public Command Elevator_BOT_BOT() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosBotBot));
  }

  public Command Elevator_BOT_TOP() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosBotTop));
  }

  public void setElevatorPos(ElevatorPosition target) {
    this.targetPosition = target;
  }

  //--------------------------------------------------------------------------
  //
  //        Elevator Motor methods
  //
  //--------------------------------------------------------------------------

  public double getElevatorPositionRads() {
    return inputs.positionRads;
  }

  private boolean isElevatorInPosition() {
    boolean isElevatorSettled = false;
    boolean isElevatorInPos = (Math.abs((getElevatorPositionRads() - targetPosition.getTargetPositionRads())) < 5);
    if(isElevatorInPos) {
      settlingCounter++;
      if(settlingCounter >= 10) {
        isElevatorSettled = true;
        settlingCounter = 0;
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
    return inputs.velocityRadsPerSec;
  }

  public void elevatorFullManual(double manualPower) {
    this.elevatorSpeed = manualPower;
    targetPosition = ElevatorPosition.PosManual;
  }

  public Command elevatorFullManual(Supplier<Double> manuaSupplier) {

    return run(() -> elevatorFullManual(manuaSupplier.get()));
  }
}
