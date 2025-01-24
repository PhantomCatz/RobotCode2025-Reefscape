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
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class CatzClimb extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private Position PositionType;
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
  public enum Position {
    ZERO(() -> 0.0),
    TUNNABLE(tunnablePos);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Position", inputs);
    // System.out.println(position);
    if (DriverStation.isDisabled()) {

    } else {
      io.setPosition(position);
    }
    Logger.recordOutput("Position/targetPosition", position);
  }

  public Command setPosition() {
    return startEnd(
        () -> position = Position.TUNNABLE.getTargetMotionPosition(),
        () -> position = Position.ZERO.getTargetMotionPosition());
  }
}
