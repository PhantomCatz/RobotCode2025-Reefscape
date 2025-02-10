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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
  private double targetManualPosition = 0.0;
  private boolean breakModeEnabled = true;
  private boolean isCharacterizing = false;


  private ElevatorFeedforward ff = new ElevatorFeedforward(gains.kS(), gains.kG(), gains.kV(), gains.kA());

  private ElevatorPosition targetPosition = ElevatorPosition.PosL1Home;

  @RequiredArgsConstructor
  public static enum ElevatorPosition {
      PosL1Home(() -> 0.0), //TBD
      PosL2(() -> 34.7),
      PosL3(() -> 80.0),
      PosL4(() -> 155.0),
      PosManual(new LoggedTunableNumber("Elevator/ScoreSourceSetpoint",0.0));

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
    Logger.processInputs("inputs/Elevator", inputs);

    //--------------------------------------------------------------------------------------------------------
    // Update controllers when user specifies
    //--------------------------------------------------------------------------------------------------------
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);
    LoggedTunableNumber.ifChanged(hashCode(),
        ()-> io.setMotionMagicParameters(mmCruiseVelocity.get(), mmAcceleration.get(), mmJerk.get()),
        mmCruiseVelocity,
        mmAcceleration,
        mmJerk);

    //---------------------------------------------------------------------------------------------------------------------------
    //    Limit switch position setting
    //---------------------------------------------------------------------------------------------------------------------------
    // if(inputs.isBotLimitSwitched) {
    //   io.setPosition(ElevatorPosition.PosL4.getTargetPositionRads());
    // } else if (inputs.isTopLimitSwitched) {
    //   io.setPosition(ElevatorPosition.PosL1Home.getTargetPositionRads());
    // }

    //---------------------------------------------------------------------------------------------------------------------------
    //    Feed Foward
    //---------------------------------------------------------------------------------------------------------------------------
    elevatorFeedForward =  gains.kG();

    //---------------------------------------------------------------------------------------------------------------------------
    //    Control Mode setting
    //---------------------------------------------------------------------------------------------------------------------------
    if(DriverStation.isDisabled()) {
      io.stop();
    } else if(targetPosition != ElevatorPosition.PosManual){
        System.out.println("setpos");
        io.runSetpoint(targetPosition.getTargetPositionRads(), elevatorFeedForward);
    } else {
      io.runSetpoint(targetManualPosition, elevatorFeedForward);
    }

    //----------------------------------------------------------------------------------------------------------------------------
    // Logging
    //----------------------------------------------------------------------------------------------------------------------------
    Logger.recordOutput("Elevator/CurrentRotations", getElevatorPositionRads());
    Logger.recordOutput("Elevator/isElevatorInPos", isElevatorInPosition());
    Logger.recordOutput("Elevator/targetPosition", targetPosition);

  }
  //--------------------------------------------------------------------------------------------------------------------------
  //
  //  Elevator Setpos Commands
  //
  //--------------------------------------------------------------------------------------------------------------------------

  public Command Elevator_L1() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL1Home));
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

  public boolean isElevatorInPosition() {
    return (Math.abs((getElevatorPositionRads() - targetPosition.getTargetPositionRads())) < 5.0);
  }

  public void setBrakeMode(boolean enabled) {
    if(breakModeEnabled == false) return;
    breakModeEnabled = true;
    io.setBrakeMode(breakModeEnabled);
  }

  public void runCharacterization(double amps) {
    isCharacterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void endCharacterization() {
    isCharacterizing = false;
  }


  public void elevatorManual(Supplier<Double> manualSupplier) {
    targetManualPosition += manualSupplier.get() * MANUAL_SCALE;
    System.out.println(targetManualPosition);
    targetPosition = ElevatorPosition.PosManual;
  }

  public Command elevatorManualMode(Supplier<Double> manualSupplier) {
    return run(() -> elevatorManual(manualSupplier)).alongWith(Commands.print("hi"));
  }

  public Command elevatorFullManual(Supplier<Double> manuaSupplier) {
    return run(()->io.runMotor(manuaSupplier.get())).alongWith(Commands.print("full manual"));
  }
}
