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
  static final double MANUAL_SCALE = 2.0;
  static double position;
  static LoggedTunableNumber tunnablePos = new LoggedTunableNumber("Climb/TunnablePosition", 1);
  static LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", 0.17);
  static LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI", 0.0);
  static LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD", 0.0006);

  static LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS", 0);
  static LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV", 0);
  static LoggedTunableNumber kA = new LoggedTunableNumber("Climb/kA", 0);


  @RequiredArgsConstructor
  public enum ClimbPosition { //In Rotations //TBD
    RETRACT(() -> -46),
    HOME(() -> 0.0),
    FULLTURN(() -> -648.0),
    MANUAL(() -> manualPow),
    FULL_MANUAL(() -> 0.0),
    TUNNABLE(tunnablePos);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }

  private ClimbPosition targetPosition = ClimbPosition.HOME;

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
    Logger.processInputs("RealInputs/Climb", inputs);
    if (DriverStation.isDisabled()) {

    } else {
      if(targetPosition == ClimbPosition.FULL_MANUAL) {
        io.setPower(manualPow);
        System.out.println("full");
      } else if(targetPosition == ClimbPosition.MANUAL) {
        io.setPosition(position);
        System.out.println("semi");
      } else if(targetPosition != ClimbPosition.MANUAL && targetPosition != ClimbPosition.FULL_MANUAL) {
        // System.out.println("Target");
        io.setPosition(position);
      } else {
        io.setPower(0.0);
      }
    }

    Logger.recordOutput("Position/targetPosition", position);
  }

  public Command Climb_Home() {
    return runOnce(() -> setClimbPos(ClimbPosition.HOME));
  }

  public Command Climb_Retract() {
    return runOnce(() -> setClimbPos(ClimbPosition.RETRACT));
  }

  public Command Climb_Full() {
    return runOnce(() -> setClimbPos(ClimbPosition.FULLTURN));
  }

  public Command Climb_Tunnable() {
    return runOnce(() -> setClimbPos(ClimbPosition.TUNNABLE));
  }

  public void setClimbPos(ClimbPosition target) {
    position = target.getTargetMotionPosition();
    targetPosition = target;
  }

  public void climbSemiManual(double manualSemiPwr) {
    double previousPos = position;
    position += manualSemiPwr * MANUAL_SCALE;
    if (Math.abs(manualSemiPwr) < 0.1) {
      position = previousPos;
    }
    // System.out.println(position);
    targetPosition = ClimbPosition.MANUAL;
  }

  public void climbFullManual(double joystickPower) {
    manualPow = joystickPower * 0.5;
    targetPosition = ClimbPosition.FULL_MANUAL;
  }

  public Command ClimbManualMode(Supplier<Double> manualSupplier) {
    return run(() -> climbFullManual(manualSupplier.get())).alongWith(Commands.print("hi"));
  }
}
