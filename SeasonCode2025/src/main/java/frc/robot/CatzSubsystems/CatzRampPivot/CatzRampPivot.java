// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzRampPivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

import static frc.robot.CatzSubsystems.CatzRampPivot.RampPivotConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class CatzRampPivot extends SubsystemBase {

  private final RampPivotIO io;
  private final RampPivotIOInputsAutoLogged inputs = new RampPivotIOInputsAutoLogged();

  public double targetPos = RampPivotPositions.PosStow.getTargetPositionRot();
  public double RampPivotFeedForward = 0.0;
  public static boolean isManual = false;
  @RequiredArgsConstructor
  public static enum RampPivotPositions {
    PosStow(() -> RAMP_STOW),
    PosClimb(() -> RAMP_CLIMB),
    Pos3(() -> heightPlaceholder),
    PosNull(() -> heightPlaceholder),
    PosManual(new LoggedTunableNumber("RampPivot/RampPivotManual",0.0));

    private final DoubleSupplier elevatorSetpointSupplier;
    private double getTargetPositionRot() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }
  public CatzRampPivot() {
        if(isRampPivotDisabled) { //Comes from Algae Remover Constants
      io = new RampPivotIONull();
      System.out.println("Ramp Pivot Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new RampPivotIOReal();
          System.out.println("Ramp Pivot Configured for Real");
        break;
        case REPLAY:
          io = new RampPivotIOReal() {};
          System.out.println("Ramp Pivot Configured for Replayed simulation");
        break;
        default:
          io = new RampPivotIONull();
          System.out.println("Ramp Pivot Unconfigured");
        break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RealInputs/RampPivot", inputs);

    if(DriverStation.isDisabled()) {
      // Disabled
      io.stop();
      targetPos = RampPivotPositions.PosStow.getTargetPositionRot();

    } else if(targetPos != RampPivotPositions.PosNull.getTargetPositionRot()){
      io.setPosition(targetPos, 0);

    } else {
      io.stop();
    }
    Logger.recordOutput("RampPivot/targetPos", targetPos);
  }

  public Command rampPivotManual(Supplier<Double> manualSupplier) {
    return run(() -> rampPivotSetManual(manualSupplier.get())).alongWith(Commands.print("full manual"));
  }

  public void rampPivotSetManual(double manualSupplier) {
    targetPos += manualSupplier * MANUAL_SCALE;

  }

  public Command Ramp_Stow() {
    return runOnce(() -> this.targetPos = RampPivotPositions.PosStow.getTargetPositionRot());
  }

  public Command Ramp_Climb() {
    return runOnce(() -> this.targetPos = RampPivotPositions.PosClimb.getTargetPositionRot());
  }
}
