// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.AlgaePivotConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class CatzAlgaePivot extends SubsystemBase {

  private final AlgaePivotIO io;
  private final AlgaePivotIOInputsAutoLogged inputs = new AlgaePivotIOInputsAutoLogged();

  private static double manualPow = 0;
  private static boolean isManual = false;
  private static int settlingCounter;
  static LoggedTunableNumber tunnablePos = new LoggedTunableNumber("AlgaePivot/TunnablePosition", 1);
  static LoggedTunableNumber kP = new LoggedTunableNumber("AlgaePivot/kP", 0.17);
  static LoggedTunableNumber kI = new LoggedTunableNumber("AlgaePivot/kI", 0.0);
  static LoggedTunableNumber kD = new LoggedTunableNumber("AlgaePivot/kD", 0.0006);

  static LoggedTunableNumber kS = new LoggedTunableNumber("AlgaePivot/kS", 0);
  static LoggedTunableNumber kV = new LoggedTunableNumber("AlgaePivot/kV", 0);
  static LoggedTunableNumber kA = new LoggedTunableNumber("AlgaePivot/kA", 0);

  /** Creates a new PositionSubsystem. */

  // ==========================================================//
  // ^^ Hallo make sure you set this to the correct motor ^^  //
  // ==========================================================//
  @RequiredArgsConstructor
  public enum Position { //In degrees
    STOW(() -> 109.0),
    HORIZONTAL(() -> -15.0), // TBD
    NetAlgae(() -> 100.0), // TBD
    MANUAL(() -> manualPow),
    BOTBOT(() -> -20.7),
    BOTTOP(() -> -20.1),
    PUNCH(() -> 30),
    TUNNABLE(tunnablePos);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
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
    // System.out.println(position);
    if (DriverStation.isDisabled()) {

    } else {
      if(isManual) {

      } else {
        io.runSetpoint(INITIAL_POSITION, 0.0);
      }
    }
    Logger.recordOutput("AlgaePivot/currentPosition", getAlgaePivotPositionRads());

    Logger.recordOutput("AlgaePivot/targetPosition", INITIAL_POSITION);

    //System.out.println("Algae Pivot FeedFowards(" + gains.kG() + " * cos(" + getAlgaePivotPositionDeg() + "): " + algaePivotFeedFoward * gains.kG());
  }

  public double getAlgaePivotPositionRads() {
    return inputs.positionDegrees;
  }

  public boolean isAlgaePivotInPosition() {
    boolean isAlgaePivotSettled = false;
    boolean isAlgaePivotInPos = (Math.abs(getAlgaePivotPositionRads() - INITIAL_POSITION) < 10);
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

  public Command AlgaePivot_Stow() {
    return runOnce(() -> setAlgaePivotPos(Position.STOW));
  }

  public Command AlgaePivot_Horizontal() {
    return runOnce(() -> setAlgaePivotPos(Position.HORIZONTAL));
  }

  public Command AlgaePivot_NetAlgae() {
    return runOnce(() -> setAlgaePivotPos(Position.NetAlgae));
  }

  public Command AlgaePivot_Tunnable() {
    return runOnce(() -> setAlgaePivotPos(Position.TUNNABLE));
  }

  public Command AlgaePivot_BotBot() {
    return runOnce(() -> setAlgaePivotPos(Position.BOTBOT));
  }

  public Command AlgaePivot_BotTop() {
    return runOnce(() -> setAlgaePivotPos(Position.BOTTOP));
  }

  public Command AlgaePivot_Punch() {
    return runOnce(() -> setAlgaePivotPos(Position.PUNCH));
  }

  public void setAlgaePivotPos(Position target) {
    INITIAL_POSITION = target.getTargetMotionPosition();
    isManual = false;
  }

  public void algaePivotManual(Supplier<Double> manualSupplier) {
    INITIAL_POSITION += manualSupplier.get() * MANUAL_SCALE;
    isManual = true;
    // System.out.println("algae:" +INITIAL_POSITION);
  }

  public Command AlgaePivotFullManualCommand(Supplier<Double> manualSupplier) {
    return run(() -> algaePivotManual(manualSupplier));
  }


}
