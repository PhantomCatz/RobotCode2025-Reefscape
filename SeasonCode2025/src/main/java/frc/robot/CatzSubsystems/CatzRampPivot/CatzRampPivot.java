package frc.robot.CatzSubsystems.CatzRampPivot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.NeutralMode;
import lombok.RequiredArgsConstructor;

import static frc.robot.CatzSubsystems.CatzRampPivot.RampPivotConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class CatzRampPivot extends SubsystemBase {
  public static final CatzRampPivot Instance = new CatzRampPivot();

  private final RampPivotIO io;
  private final RampPivotIOInputsAutoLogged inputs = new RampPivotIOInputsAutoLogged();

  private RampPivotPositions rampPivotPositions = RampPivotPositions.PosStow;
  private double rampPower = 0.0;

  private double targetPos = RampPivotPositions.PosStow.getTargetPositionRot();
  private double RampPivotFeedForward = 0.0;
  private static boolean isManual = false;
  private BooleanSupplier manualSupplier = ()-> false;

  public NeutralMode currentNeutralMode = NeutralMode.COAST;
  public NeutralMode prevNeutralMode = NeutralMode.COAST;

  @RequiredArgsConstructor
  public static enum RampPivotPositions {
    PosStow(() -> RAMP_STOW),
    PosClimb(() -> RAMP_CLIMB),
    PosIntake(() -> RAMP_INTAKE),
    PosNull(() -> heightPlaceholder),
    PosManual(new LoggedTunableNumber("RampPivot/RampPivotManual",0.0));

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getTargetPositionRot() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }
  private CatzRampPivot() {
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
      targetPos = RampPivotPositions.PosIntake.getTargetPositionRot();


    } else if (manualSupplier.getAsBoolean() || rampPivotPositions == RampPivotPositions.PosManual) {
      io.runMotor(rampPower);

    } else if(rampPivotPositions != RampPivotPositions.PosNull &&
              rampPivotPositions != RampPivotPositions.PosManual){

    //  System.out.println("RaMp_PiVoT TaRgEt: " + targetPos);
      io.setPosition(targetPos, 0);

    } else {
      System.out.println("no power");
      io.stop();
    }
    Logger.recordOutput("RampPivot/targetPos", targetPos);
    Logger.recordOutput("RampPivot/targetPosEnum", rampPivotPositions.toString());

  }

  public Command rampPivotManual(Supplier<Double> manualSupplier) {
    return run(() -> rampPivotSetManual(manualSupplier.get())).alongWith(Commands.print("full manual"));
  }

  public Command rampPivotPosManual(Supplier<Double> manualSupplier) {
    return runOnce(() -> rampPivotPosManual(manualSupplier.get())).alongWith(Commands.print("pos manual"));
  }

  public void setNeutralMode(NeutralMode mode) {
    io.setNeutralMode(mode);
  }

  public void rampPivotSetManual(double manualSupplier) {
    this.rampPower = manualSupplier * 0.1;
    rampPivotPositions = RampPivotPositions.PosManual;
  }

  public void rampPivotPosManual(double manualSupplier) {
    targetPos += manualSupplier * MANUAL_SCALE;
  }

   public void setRampPos(RampPivotPositions target) {
    this.targetPos = target.getTargetPositionRot();
  }

  public boolean isSafeToRaiseElevator(){
    return inputs.positionMechs >= RampPivotConstants.RAMP_INTAKE - 0.2;
  }

  public Command Ramp_Stow_Pos() {
    return runOnce(() -> setRampPos(RampPivotPositions.PosStow));
  }

  public Command Ramp_Intake_Pos() {
    return runOnce(() -> setRampPos(RampPivotPositions.PosIntake));
  }

  public Command Ramp_Climb_Pos() {
    return runOnce(() -> setRampPos(RampPivotPositions.PosClimb));
  }

  public void setOverride(BooleanSupplier manualSupplier) {
    this.manualSupplier = manualSupplier;
  }
}
