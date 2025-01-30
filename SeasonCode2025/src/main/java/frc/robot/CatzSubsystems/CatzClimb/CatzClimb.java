// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzClimb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class CatzClimb extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private Position PositionType;
  static double manualPow = 0;
  static boolean isManual;
  static final double MANUAL_SCALE = 5;
  static double position;
  static LoggedTunableNumber tunnablePos = new LoggedTunableNumber("Climb/TunnablePosition", 1);
  static LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", 0.17);
  static LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI", 0.0);
  static LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD", 0.0006);

  static LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS", 0);
  static LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV", 0);
  static LoggedTunableNumber kA = new LoggedTunableNumber("Climb/kA", 0);

  /** Creates a new PositionSubsystem. */
  public CatzClimb() {
    // io = new PositionIOKraken() {};
    io = new ClimbIOReal() {};

    io.setPID(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
  }

  // ==========================================================//
  // ^^ Hallo make sure you set this to the correct motor ^^  //
  // ==========================================================//
  @RequiredArgsConstructor
  public enum Position { //In degrees
    RETRACT(() -> -46),
    HOME(() -> 0.0),
    STOW(() -> 40.0),
    FULLTURN(() -> 90),
    MANUAL(() -> manualPow),
    TUNNABLE(tunnablePos);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb/inputs", inputs);
    // System.out.println(position);
    if (DriverStation.isDisabled()) {

    } else {
      // if(isManual) {
      //   io.setPower(manualPow);
      // } else {
        io.setPosition(position);
      // }
    }
    Logger.recordOutput("Position/targetPosition", position);
  }

  public Command Climb_Stow() {
    return runOnce(() -> setClimbPos(Position.STOW));
  }

  public Command Climb_Home() {
    return runOnce(() -> setClimbPos(Position.HOME));
  }

  public Command Climb_Retract() {
    return runOnce(() -> setClimbPos(Position.RETRACT));
  }

  public Command Climb_Full() {
    return runOnce(() -> setClimbPos(Position.FULLTURN));
  }

  public Command Climb_Tunnable() {
    return runOnce(() -> setClimbPos(Position.TUNNABLE));
  }

  public void setClimbPos(Position target) {
    position = target.getTargetMotionPosition();
    isManual = false;
  }

  public void climbManual(Supplier<Double> manualSupplier) {
    // manualPow = manualSupplier.get();
    // isManual = true;
    position += manualSupplier.get() * MANUAL_SCALE;
    System.out.println(position);
  }

  public Command ClimbManualMode(Supplier<Double> manualSupplier) {
    return run(() -> climbManual(manualSupplier));
  }
}
