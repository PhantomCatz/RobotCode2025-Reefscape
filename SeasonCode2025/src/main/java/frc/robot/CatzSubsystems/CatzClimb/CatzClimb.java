// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzClimb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;

import static frc.robot.CatzSubsystems.CatzClimb.ClimbConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class CatzClimb extends SubsystemBase {

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

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

  @RequiredArgsConstructor
  public enum Position { //In degrees
    RETRACT(() -> -46),
    HOME(() -> 0.0),
    FULLTURN(() -> 90),
    MANUAL(() -> manualPow),
    TUNNABLE(tunnablePos);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }

  public CatzClimb() {
    if(isClimbDisabled) { //Comes from Climb Constants
      io = new ClimbIONull();
      System.out.println("Climb Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new ClimbIOReal();
          System.out.println("Climb Configured for Real");
        break;
        case REPLAY:
          io = new ClimbIOReal() {};
          System.out.println("Climb Configured for Replayed simulation");
        break;
        default:
          io = new ClimbIONull();
          System.out.println("Climb Unconfigured");
        break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb/inputs", inputs);
    if (DriverStation.isDisabled()) {

    } else {
      if(isManual == false) {
        io.setPosition(position);
      } else {

      }
    }
    Logger.recordOutput("Position/targetPosition", position);
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

  public void climbSemiManual(Supplier<Double> manualSupplier) {
    position += manualSupplier.get() * MANUAL_SCALE;
    // System.out.println(position);
    isManual = false;
  }

  public void climbFullManual(double joystickPower) {
    io.setPower(joystickPower);
    isManual = true;
  }

  public Command ClimbManualMode(Supplier<Double> manualSupplier) {
    return run(() -> climbFullManual(manualSupplier.get())).alongWith(Commands.print("hi"));
  }
}
