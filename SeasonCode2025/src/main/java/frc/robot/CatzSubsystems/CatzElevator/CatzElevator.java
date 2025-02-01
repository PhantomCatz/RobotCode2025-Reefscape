// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
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

  private static LoggedTunableNumber tunableNumber =
      new LoggedTunableNumber("Elevator/MotorPower", 0.1);

  private double elevatorSpeed = 0.0;
  private boolean breakModeEnabled = true;
  private boolean isCharacterizing = false;


  private ElevatorPosition targetPosition = ElevatorPosition.PosL1Home;

  private ElevatorFeedforward ff;

  @RequiredArgsConstructor
  public static enum ElevatorPosition {
      PosL1Home(() -> 25.0), //TBD
      PosL2(() -> 50.0),
      PosL3(() -> 75.0),
      PosL4(() -> 100.0),
      PosManual(new LoggedTunableNumber("Elevator/ScoreSourceSetpoint",0.0));

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getTargetPositionRotations() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  public CatzElevator() {
    if(isElevatorDisabled) { //Comes from elevator Constants
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
    Logger.processInputs("Elevator/inputs", inputs);

    // Update controllers when user specifies
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

    if(DriverStation.isDisabled() || targetPosition == null) {
      io.stop();
    } else {
      io.runSetpoint(targetPosition.getTargetPositionRotations(), ff.calculate(inputs.velocityRpm));
    }

    Logger.recordOutput("Elevator/CurrentRotations", getElevatorPositionRotations());
    Logger.recordOutput("Elevator/isElevatorInPos", isElevatorInPosition());

  }


  //--------------------------------------------------------------------------
  //
  //
  //
  //--------------------------------------------------------------------------

  public double getElevatorPositionRotations() {
    return inputs.positionRotations;
  }

  public boolean isElevatorInPosition() {
    return (Math.abs((getElevatorPositionRotations() - targetPosition.getTargetPositionRotations())) < 5.0);
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
    return inputs.velocityRpm;
  }

  public void endCharacterization() {
    isCharacterizing = false;
  }

  public void setTargetPosition(ElevatorPosition targetPosition) {
    this.targetPosition = targetPosition;
  }

  public Command setTargetPositionCommand (ElevatorPosition targetPosition) {
    return runOnce(() -> setTargetPosition(targetPosition));
  }


}
