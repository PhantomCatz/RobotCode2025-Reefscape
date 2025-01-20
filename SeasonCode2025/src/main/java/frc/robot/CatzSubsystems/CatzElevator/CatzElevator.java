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

import org.littletonrobotics.junction.Logger;

public class CatzElevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static LoggedTunableNumber tunableNumber =
      new LoggedTunableNumber("Elevator/MotorPower", 0.1);

  private double elevatorSpeed = 0.0;

  @RequiredArgsConstructor
  public static enum ElevatorPosition {
      PosL1Home(() -> 25.0),
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
          io = null;
          System.out.println("Elevator Unconfigured");
        break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("elevator", inputs);

    io.runMotor(elevatorSpeed); // System.out.println(tunableNumber.get());
  }

  public Command setElevatorPos() {
    return runOnce(() ->  = tunableNumber.getAsDouble(), () -> elevatorSpeed = 0);
  }

  public Command runMotorBck() {
    return startEnd(() -> elevatorSpeed = tunableNumber.getAsDouble(), () -> elevatorSpeed = 0);
  }
}
