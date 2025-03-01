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

  static double manualPow = 0;
  static boolean isManual;
  static final double MANUAL_SCALE = 5;
  static double position = 0.0;
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
    HORIZONTAL(() -> -30.0), // TBD
    UNDISCLOSED(() -> 999), // TBD
    MANUAL(() -> manualPow),
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
        io.runSetpoint(position, 0.0);
      }
    }
    Logger.recordOutput("AlgaePivot/currentPosition", getAlgaePivotPositionRads());

    Logger.recordOutput("AlgaePivot/targetPosition", position);

  }

  public double getAlgaePivotPositionRads() {
    return inputs.positionMechs;
  }

  public Command AlgaePivot_Stow() {
    return runOnce(() -> setAlgaePivotPos(Position.STOW));
  }

  public Command AlgaePivot_Horizontal() {
    return runOnce(() -> setAlgaePivotPos(Position.HORIZONTAL));
  }

  public Command AlgaePivot_Undisclosed() {
    return runOnce(() -> setAlgaePivotPos(Position.UNDISCLOSED));
  }

  public Command AlgaePivot_Tunnable() {
    return runOnce(() -> setAlgaePivotPos(Position.TUNNABLE));
  }

  public void setAlgaePivotPos(Position target) {
    position = target.getTargetMotionPosition();
    // System.out.println(position);
    isManual = false;
  }

  public void algaePivotManual(Supplier<Double> manualSupplier) {
    position += manualSupplier.get() * MANUAL_SCALE;
    System.out.println("algae:" +position);
  }


  public Command AlgaePivotFullManual(Supplier<Double> manualSupplier) {
    return run(() -> algaePivotManual(manualSupplier));
  }

  public void algaePivotFullManual(Supplier<Double> manualSupplier) {
    io.setPercentOutput(manualSupplier.get()/10);
  }

  public Command AlgaePivotFullManualCommand(Supplier<Double> manualSupplier) {
    return run(() -> algaePivotFullManual(manualSupplier));
  }


}
