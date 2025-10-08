package frc.robot.CatzSubsystems.CatzClimb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.WinchingState;
import frc.robot.Utilities.LoggedTunableNumber;

import static frc.robot.CatzSubsystems.CatzClimb.ClimbConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class CatzClimb extends SubsystemBase {
  public static final CatzClimb Instance = new CatzClimb();

  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  static double manualPow = 0;
  static boolean isManual = false;
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
  public enum ClimbPosition { //In Rotations
    RETRACT(() -> 0.0),
    HOME(() -> 0.0),
    EXTENDING(() -> -330.0),
    MANUAL(() -> 0.0),
    FULL_MANUAL(() -> 0.0),
    TUNNABLE(tunnablePos);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }

  private ClimbPosition targetPosition = ClimbPosition.FULL_MANUAL;

  private CatzClimb() {
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
      io.setPower(0.0);
      manualPow = 0.0;
      // targetPosition = ClimbPosition.MANUAL;
      targetPosition = ClimbPosition.FULL_MANUAL;

    } else {
      if(isManual || targetPosition == ClimbPosition.FULL_MANUAL) {
        io.setPower(manualPow);
        System.out.println(manualPow);
        //System.out.println("full");
      } else if(targetPosition == ClimbPosition.MANUAL) {
        io.setPosition(position);
        // System.out.println("semi");
      } else if(targetPosition != ClimbPosition.MANUAL && targetPosition != ClimbPosition.FULL_MANUAL) {
        //System.out.println("Target + " + position);
        io.setPosition(position);
      } else {
        io.setPower(0.0);
      }

      if(inputs.commandedOutput > 0.1) {
        CatzLED.Instance.setClimbDirection(WinchingState.EXTENDING);
      } else if(inputs.commandedOutput < -0.1) {
        CatzLED.Instance.setClimbDirection(WinchingState.RETRACTING);
      } else {
        CatzLED.Instance.setClimbDirection(WinchingState.IDLE);
      }
    }

    Logger.recordOutput("Climb/targetPosition", position);
  }

  public Command Climb_Home() {
    return runOnce(() -> setClimbPos(ClimbPosition.HOME));
  }

  public Command reZero(){
    return runOnce(() -> io.setZero());
  }

  public Command fullClimb() {
    return runOnce(() -> setClimbPos(ClimbPosition.RETRACT));
  }

  public Command extendClimb() {
    return runOnce(() -> setClimbPos(ClimbPosition.EXTENDING)).alongWith(new PrintCommand("Extending Climb!"));
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
    manualPow = joystickPower;
    targetPosition = ClimbPosition.FULL_MANUAL;
  }

  public Command ClimbManualMode(Supplier<Double> manualSupplier) {
    return run(() -> climbFullManual(manualSupplier.get())).alongWith(Commands.print("hi")).onlyIf(() -> CatzSuperstructure.isClimbEnabled());
  }

  public Command ClimbManualModeAux(Supplier<Double> manualSupplier) {
    return run(() -> climbFullManual(manualSupplier.get()));
  }

  public Command CancelClimb() {
    Command cancel = Commands.runOnce(()->manualPow = 0.0);
    cancel.addRequirements(this);
    return cancel;
  }

  public void setOverrides(Supplier<Boolean> booleanSupplier) {
    isManual = booleanSupplier.get();
  }

}
