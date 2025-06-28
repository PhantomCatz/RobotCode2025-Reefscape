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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.AlgaePivotConstants.*;


/** Add your docs here. */
public class AlgaePivotIOReal implements AlgaePivotIO {

  private TalonFX algaePivotMotor = new TalonFX(ALGAE_PIVOT_MOTOR_ID);

  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // private final PIDController pid = new PIDController(slot0_gains.kP(), slot0_gains.kI(), slot0_gains.kD());

  private final StatusSignal<Angle> algaePivotPosition;
  private final StatusSignal<AngularVelocity> algaePivotVelocity;
  private final StatusSignal<Voltage> algaePivotAppliedVolts;
  private final StatusSignal<Current> algaePivotSupplyCurrent;
  private final StatusSignal<Current> algaePivotTorqueCurrent;
  private final StatusSignal<Temperature> algaePivotTempCelsius;

    public AlgaePivotIOReal() {
      algaePivotPosition = algaePivotMotor.getPosition();
      algaePivotVelocity = algaePivotMotor.getVelocity();
      algaePivotAppliedVolts = algaePivotMotor.getMotorVoltage();
      algaePivotSupplyCurrent = algaePivotMotor.getSupplyCurrent();
      algaePivotTorqueCurrent = algaePivotMotor.getTorqueCurrent();
      algaePivotTempCelsius = algaePivotMotor.getDeviceTemp();

      BaseStatusSignal.setUpdateFrequencyForAll(
          100.0,
          algaePivotPosition,
          algaePivotVelocity,
          algaePivotAppliedVolts,
          algaePivotSupplyCurrent,
          algaePivotTorqueCurrent,
          algaePivotTempCelsius);

      config.Slot0.kP = slot0_gains.kP();
      config.Slot0.kI = slot0_gains.kI();
      config.Slot0.kD = slot0_gains.kD();
      config.Slot0.kS = slot0_gains.kS();
      config.Slot0.kV = slot0_gains.kV();
      config.Slot0.kA = slot0_gains.kA();


      config.Slot1.kP = slot1_gains.kP();
      config.Slot1.kI = slot1_gains.kI();
      config.Slot1.kD = slot1_gains.kD();
      config.Slot1.kS = slot1_gains.kS();
      config.Slot1.kV = slot1_gains.kV();
      config.Slot1.kA = slot1_gains.kA();

      config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


      config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
      config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
      config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();

      algaePivotMotor.getConfigurator().apply(config, 1.0);

      algaePivotMotor.setPosition(PIVOT_INITIAL_POS);
    }

  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    inputs.isPositionIOMotorConnected =
        BaseStatusSignal.refreshAll(
                algaePivotPosition,
                algaePivotVelocity,
                algaePivotAppliedVolts,
                algaePivotSupplyCurrent,
                algaePivotTorqueCurrent,
                algaePivotTempCelsius)
            .isOK();
    inputs.positionDegrees     = (algaePivotPosition.getValueAsDouble() / ALGAE_PIVOT_GEAR_REDUCTION) * 360.0;
    inputs.velocityRpm         = (algaePivotVelocity.getValueAsDouble() * 60.0) / ALGAE_PIVOT_GEAR_REDUCTION * 360.0;
    inputs.appliedVolts        = algaePivotAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps   = algaePivotSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps   = algaePivotTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius         = algaePivotTempCelsius.getValueAsDouble();
  }

  @Override
  public void resetPosition(double pos) {// Set the motor position in mechanism rotations
    algaePivotMotor.setPosition((pos / 360.0) * ALGAE_PIVOT_GEAR_REDUCTION);
  }

  @Override
  public void runSetpointUp(double targetDegrees, double feedforwardVolts) {

    double setpointRotations = ((targetDegrees / 360.0) * ALGAE_PIVOT_GEAR_REDUCTION);
    algaePivotMotor.setControl(positionControl.withPosition(setpointRotations)
                                              .withFeedForward(feedforwardVolts));
  }

  @Override
  public void runSetpointDown(double targetDegrees, double feedforwardVolts) {
    double setpointRotations = ((targetDegrees / 360.0) * ALGAE_PIVOT_GEAR_REDUCTION);
    algaePivotMotor.setControl(positionControl.withPosition(setpointRotations)
                                              .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
    algaePivotMotor.getConfigurator().apply(config);
  }

  @Override
  public void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot1.kP = kP;
    config.Slot1.kI = kI;
    config.Slot1.kD = kD;
    config.Slot1.kS = kS;
    config.Slot1.kV = kV;
    config.Slot1.kA = kA;
    System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
    algaePivotMotor.getConfigurator().apply(config);
  }

  @Override
  public void runCharacterizationMotor(double input) {
    algaePivotMotor.setControl(voltageControl.withOutput(input));
  }

  @Override
  public void setPercentOutput(double percentOutput) {
    algaePivotMotor.setControl(new DutyCycleOut(percentOutput));
  }
}
