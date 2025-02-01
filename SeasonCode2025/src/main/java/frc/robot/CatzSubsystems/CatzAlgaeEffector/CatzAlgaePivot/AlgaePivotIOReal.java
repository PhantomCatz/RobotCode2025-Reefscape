// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.AlgaePivotConstants.*;


/** Add your docs here. */
public class AlgaePivotIOReal implements AlgaePivotIO {

  TalonFX algaePivotMotor = new TalonFX(ALGAE_PIVOT_MOTOR_ID);


  private final PositionVoltage positionControl = new PositionVoltage(0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

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

    config.Slot0.kP = 12.0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.MotionMagic.MotionMagicAcceleration = 400;
    config.MotionMagic.MotionMagicJerk = 1600;

    algaePivotMotor.getConfigurator().apply(config, 1.0);

    algaePivotMotor.setPosition(0);
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
    inputs.positionMechs = algaePivotPosition.getValueAsDouble();
    inputs.velocityRpm = algaePivotVelocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = algaePivotAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = algaePivotSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = algaePivotTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius = algaePivotTempCelsius.getValueAsDouble();
  }

  @Override
  public void setPosition(double pos) // Set the motor position in mechanism rotations
      {
    CatzAlgaePivot.position = pos;
    algaePivotMotor.setControl(positionControl.withPosition(pos));
    // System.out.println(pos);
  }

  @Override
  public void setPower(double power) {
    algaePivotMotor.set(power);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
    algaePivotMotor.getConfigurator().apply(config);
  }

  @Override
  public void runCharacterizationMotor(double input) {
    algaePivotMotor.setControl(voltageControl.withOutput(input));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    System.out.println("kS: " + kS + " kV: " + kV + " kA: " + kA);
    algaePivotMotor.getConfigurator().apply(config);
  }

}
