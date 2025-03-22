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
package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.AlgaePivotConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class CatzAlgaePivot extends SubsystemBase {

  private final AlgaePivotIO io;
  private final AlgaePivotIOInputsAutoLogged inputs = new AlgaePivotIOInputsAutoLogged();
  private BooleanSupplier manualOverride = () -> false;

  private static double manualPow = 0;
  private static int settlingCounter;

  private static AlgaePivotPosition targetPosition = AlgaePivotPosition.STOW;

  @RequiredArgsConstructor
  public enum AlgaePivotPosition { //In degrees
    STOW(() -> 60.0),
    HORIZONTAL(() -> -15.0), // TBD
    NetAlgae(() -> 80.0), // TBD
    MANUAL(() -> manualPow),
    BOTBOT(() -> 0.7),
    BOTTOP(() -> 0.1),
    PUNCH(() -> 30.0),
    NULL(() -> 90.0),
    TUNNABLE(tunnablePos);

    private final DoubleSupplier motionType;

    private double getTargetAngle() {
      return motionType.getAsDouble();
    }
  }

  public CatzAlgaePivot() {
    if(isAlgaePivotDisabled) { //Comes from Algae Pivot Constants
      io = new AlgaePivotIONull();
      System.out.println("Algae Pivot Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new AlgaePivotIOReal();
          System.out.println("Algae Pivot Configured for Real");
        break;
        case REPLAY:
          io = new AlgaePivotIOReal() {};
          System.out.println("Algae Pivot Configured for Replayed simulation");
        break;
        default:
          io = new AlgaePivotIONull();
          System.out.println("Algae Pivot Unconfigured");
        break;
      }
    }

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RealInputs/AlgaePivot", inputs);

    //-------------------------------------------------------------------------------------------
    //  Tunnable Numbers
    //-------------------------------------------------------------------------------------------
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGainsSlot0(slot0_kP.get(),
                               slot0_kI.get(),
                               slot0_kD.get(),
                               slot0_kS.get(),
                               slot0_kV.get(),
                               slot0_kA.get()
        ),
        slot0_kP,
        slot0_kI,
        slot0_kD,
        slot0_kS,
        slot0_kV,
        slot0_kA
    );

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGainsSlot1(slot1_kP.get(),
                               slot1_kI.get(),
                               slot1_kD.get(),
                               slot1_kS.get(),
                               slot1_kV.get(),
                               slot1_kA.get()
        ),
        slot1_kP,
        slot1_kI,
        slot1_kD,
        slot1_kS,
        slot1_kV,
        slot1_kA
    );

    //-------------------------------------------------------------------------------------------
    //  Setpoint running
    //-------------------------------------------------------------------------------------------
    if (DriverStation.isDisabled()) {
      io.setPercentOutput(0.0);
      targetPosition = AlgaePivotPosition.NULL;
    } else {
      io.setVoltage(tunnablePos.get());
      // if(manualOverride.getAsBoolean()) {
      //   io.setPercentOutput(manualPow);
      // } else if(targetPosition != AlgaePivotPosition.NULL) {
      //   io.runSetpoint(targetPosition.getTargetAngle(), calculateArmFeedFoward());
      // } else {

      // }
    }

    // Logging
    Logger.recordOutput("Algae Pivot/targetPositionDeg", targetPosition.getTargetAngle());
    Logger.recordOutput("Algae Pivot/ff power", calculateArmFeedFoward());
  }

  //-------------------------------------------------------------------------------------------
  //  MISC methods
  //-------------------------------------------------------------------------------------------
  public double getAlgaePivotDeg() {
    return inputs.positionDegrees;
  }

  public double calculateArmFeedFoward() {
    double result = 0.0;
    result = slot0_gains.kG() * Math.cos(getAlgaePivotDeg());
    return result;
  }

  public boolean isAlgaePivotInPosition() {
    boolean isAlgaePivotSettled = false;
    boolean isAlgaePivotInPos = (Math.abs(getAlgaePivotDeg() - targetPosition.getTargetAngle()) < 10);
    if(isAlgaePivotInPos) {
      settlingCounter++;
      if(settlingCounter >= 10) {
        isAlgaePivotSettled = true;
        settlingCounter = 0;
      }

    } else {
      isAlgaePivotSettled = false;
      settlingCounter = 0;
    }
    return isAlgaePivotSettled;
  }

  public void algaePivotManual(Supplier<Double> manualSupplier) {
    // targetPosDeg += manualSupplier.get() * MANUAL_SCALE;
    //  System.out.println("algae:" +INITIAL_POSITION);
  }

  public Command AlgaePivotFullManualCommand(Supplier<Double> manualSupplier) {
    return run(() -> algaePivotManual(manualSupplier));
  }

  public void setOverrides(BooleanSupplier manualOverride) {
      this.manualOverride = manualOverride;
  }


  //-------------------------------------------------------------------------------------------
  //  Command type methods
  //-------------------------------------------------------------------------------------------
  public Command AlgaePivot_Stow() {
    return runOnce(() -> setAlgaePivotPos(AlgaePivotPosition.STOW));
  }

  public Command AlgaePivot_Horizontal() {
    return runOnce(() -> setAlgaePivotPos(AlgaePivotPosition.HORIZONTAL));
  }

  public Command AlgaePivot_NetAlgae() {
    return runOnce(() -> setAlgaePivotPos(AlgaePivotPosition.NetAlgae));
  }

  public Command AlgaePivot_Tunnable() {
    return runOnce(() -> setAlgaePivotPos(AlgaePivotPosition.TUNNABLE));
  }

  public Command AlgaePivot_BotBot() {
    return runOnce(() -> setAlgaePivotPos(AlgaePivotPosition.BOTBOT));
  }

  public Command AlgaePivot_BotTop() {
    return runOnce(() -> setAlgaePivotPos(AlgaePivotPosition.BOTTOP));
  }

  public Command AlgaePivot_Punch() {
    return runOnce(() -> setAlgaePivotPos(AlgaePivotPosition.PUNCH));
  }

  public void setAlgaePivotPos(AlgaePivotPosition target) {
    targetPosition = target;
  }

}
