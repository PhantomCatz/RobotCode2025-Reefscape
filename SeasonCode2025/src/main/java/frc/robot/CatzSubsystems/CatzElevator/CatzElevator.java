// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.Logger;



public class CatzElevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  static LoggedTunableNumber tunableNumber = new LoggedTunableNumber("Elevator/MotorPower", 0.1);
  static LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.17);
  static LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
  static LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0006);

  static LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0);
  static LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0);
  static LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0);

  private double elevatorSpeed = 0.0;
  private double elevatorFeedForward = 0.0;
  private int settlingCounter = 0;
  private boolean breakModeEnabled = true;
  private BooleanSupplier manualOverride = () -> false;


  private ElevatorPosition targetPosition = ElevatorPosition.PosStow;
  private ElevatorPosition prevTargetPositon = ElevatorPosition.PosNull;
  private ElevatorPosition previousLoggedPosition = ElevatorPosition.PosNull;

  // Trap profile
  private TrapezoidProfile profile;
  private TrapezoidProfile algaeProfile;
  @Getter private State setpoint = new State();
  private State currentGoal = new State();

  @RequiredArgsConstructor
  public static enum ElevatorPosition {
      //TO CHANGE HEIGHT GO TO ElevatorConstants.java
      PosLimitSwitchStow(() -> 0.0),
      PosStow(() -> STOW_HEIGHT),
      PosL1(() -> L1_HEIGHT),
      PosL2(() -> L2_HEIGHT),
      PosL3(() -> L3_HEIGHT),
      PosL4(() -> L4_HEIGHT),
      PosL4Adj(() -> L4_CORAL_ADJ),
      PosBotBot(() -> BOT_BOT_ALGAE),
      PosBotTop(() -> BOT_TOP_ALGAE),
      PosManual(new LoggedTunableNumber("Elevator/ScoreSourceSetpoint",0.0)),
      PosNull(() -> -1.0);

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getTargetPositionRads() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  public CatzElevator() {
    if(isElevatorDisabled) {
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

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                motionMagicParameters.mmCruiseVelocity(), motionMagicParameters.mmAcceleration()));

    algaeProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                motionMagicParameters.mmCruiseVelocity(), motionMagicParameters.mmAcceleration()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RealInputs/Elevator", inputs);

    //--------------------------------------------------------------------------------------------------------
    // Update controllers when user specifies
    //--------------------------------------------------------------------------------------------------------
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGainsSlot0(slot0_kP.get(),
                               slot0_kI.get(),
                               slot0_kD.get(),
                               slot0_kS.get(),
                               slot0_kV.get(),
                               slot0_kA.get(),
                               slot0_kG.get()
        ),
        slot0_kP,
        slot0_kI,
        slot0_kD,
        slot0_kS,
        slot0_kV,
        slot0_kA,
        slot0_kG
    );

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setGainsSlot0(slot0_kP.get(),
                               slot0_kI.get(),
                               slot0_kD.get(),
                               slot0_kS.get(),
                               slot0_kV.get(),
                               slot0_kA.get(),
                               slot0_kG.get()
        ),
        slot0_kP,
        slot0_kI,
        slot0_kD,
        slot0_kS,
        slot0_kV,
        slot0_kA,
        slot0_kG
    );

    LoggedTunableNumber.ifChanged(hashCode(),
        ()-> io.setMotionMagicParameters(mmCruiseVelocity.get(), mmAcceleration.get(), mmJerk.get()),
        mmCruiseVelocity,
        mmAcceleration,
        mmJerk);

    if(previousLoggedPosition != targetPosition) {
      prevTargetPositon = targetPosition;
    }

    //---------------------------------------------------------------------------------------------------------------------------
    //    Limit switch position setting
    //---------------------------------------------------------------------------------------------------------------------------
    if(inputs.isBotLimitSwitched) {
      io.resetPosition(ElevatorPosition.PosLimitSwitchStow.getTargetPositionRads());
    }

    //---------------------------------------------------------------------------------------------------------------------------
    //    Control Mode setting
    //---------------------------------------------------------------------------------------------------------------------------
    if(DriverStation.isDisabled()) {
      // Disabled
      io.stop();
      targetPosition = ElevatorPosition.PosNull;
    } else if(targetPosition != ElevatorPosition.PosNull &&
              targetPosition != ElevatorPosition.PosManual){
      // Setpoint PID
      if(targetPosition == ElevatorPosition.PosStow) {
        // Safety Stow
        if(getElevatorPositionRads() < 29.0) {
          io.stop();
        } else {
          io.runSetpointDown(targetPosition.getTargetPositionRads());
        }
      } else {
        //Setpoint PID
        io.runSetpointUp(targetPosition.getTargetPositionRads());
      }
    } else if (manualOverride.getAsBoolean()) {
      io.runMotor(elevatorSpeed);
    } else {
      // Nothing happening
      io.stop();
    }

    //----------------------------------------------------------------------------------------------------------------------------
    // Logging
    //----------------------------------------------------------------------------------------------------------------------------
    // Log state
    Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
    Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);

    Logger.recordOutput("Elevator/CurrentRadians", getElevatorPositionRads());
    Logger.recordOutput("Elevator/prevtargetPosition", prevTargetPositon.getTargetPositionRads());
    Logger.recordOutput("Elevator/logged prev targetPosition", previousLoggedPosition.getTargetPositionRads());
    Logger.recordOutput("Elevator/isElevatorInPos", isElevatorInPosition());
    Logger.recordOutput("Elevator/targetPosition", targetPosition.getTargetPositionRads());

    // Target Postioin Logging
    previousLoggedPosition = targetPosition;
  }
  //-------------------------------------------------------------------------------------------------------------------------
  //
  //  Setposition Control
  //
  //--------------------------------------------------------------------------------------------------------------------------

  private void runTrapProfile(ElevatorPosition setPosition) {
      // Clamp goal aka where we want the elevator to be at the end
      var goalState = new State(
                            MathUtil.clamp(setPosition.getTargetPositionRads(),0.0, ElevatorConstants.MAX_TRAVEL_RADIANS),
                            0.0
      );

      double previousVelocity = inputs.velocityRadsPerSec;

      // Calculate trap sepoint
      if(setPosition == ElevatorPosition.PosBotBot ||
         setPosition == ElevatorPosition.PosBotTop ) {
        setpoint = algaeProfile.calculate(CatzConstants.LOOP_TIME, setpoint, goalState);
      } else {
        setpoint = profile.calculate(CatzConstants.LOOP_TIME, setpoint, goalState);
      }

      // Clamp trap profile
      if (setPosition.getTargetPositionRads() < 0.0 || setPosition.getTargetPositionRads() > ElevatorConstants.MAX_TRAVEL_RADIANS) {
        setpoint = new State(
                MathUtil.clamp(setpoint.position, 0.0, ElevatorConstants.MAX_TRAVEL_RADIANS),
                0.0);
      }

      System.out.println(setpoint.position);
    double accel = (setpoint.velocity - previousVelocity) / CatzConstants.LOOP_TIME;
          // io.runSetpoint(
          //     setpoint.position,
          //     ElevatorConstants.gains.kV() * Math.signum(setpoint.velocity) // Magnitude irrelevant
          //         + ElevatorConstants.gains.kG()
          //         + ElevatorConstants.gains.kA() * accel);

    currentGoal = goalState;

  }
  //--------------------------------------------------------------------------------------------------------------------------
  //
  //  Elevator Setpos Commands
  //
  //--------------------------------------------------------------------------------------------------------------------------
  public Command Elevator_Stow() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosStow));
  }

  public Command Elevator_L1() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL1));
  }

  public Command Elevator_L2() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL2));
  }

  public Command Elevator_L3() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL3));
  }

  public Command Elevator_L4() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL4));
  }

  public Command Elevator_L4_Adj() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosL4Adj));
  }

  public Command Elevator_BOT_BOT() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosBotBot));
  }

  public Command Elevator_BOT_TOP() {
    return runOnce(() -> setElevatorPos(ElevatorPosition.PosBotTop));
  }

  public void setElevatorPos(ElevatorPosition target) {
    this.targetPosition = target;
  }

  //--------------------------------------------------------------------------
  //
  //        Elevator Motor methods
  //
  //--------------------------------------------------------------------------

  public double getElevatorPositionRads() {
    return inputs.positionRads;
  }

  public boolean isElevatorInPosition() {
    boolean isElevatorSettled = false;
    boolean isElevatorInPos = (Math.abs((getElevatorPositionRads() - targetPosition.getTargetPositionRads())) < 5);
    if(isElevatorInPos) {
      settlingCounter++;
      if(settlingCounter >= 10) {
        isElevatorSettled = true;
        settlingCounter = 0;
         //System.out.println("////////////ELEVATOR SETTLED FOR .2 SECONDS///////////////////");
      }
    } else {
      isElevatorSettled = false;
      settlingCounter = 0;
    }
    return isElevatorSettled;
  }

  public void setBrakeMode(boolean enabled) {
    if(breakModeEnabled == false) return;
    breakModeEnabled = true;
    io.setBrakeMode(breakModeEnabled);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void elevatorFullManual(double manualPower) {
    this.elevatorSpeed = manualPower;
    targetPosition = ElevatorPosition.PosManual;
  }

  public void setOverrides(BooleanSupplier manualOverride) {
    this.manualOverride = manualOverride;
  }

  public Command elevatorFullManual(Supplier<Double> manuaSupplier) {

    return run(() -> elevatorFullManual(manuaSupplier.get()));
  }
}
