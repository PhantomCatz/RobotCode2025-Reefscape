// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzClimb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimbIOReal implements ClimbIO {

  TalonFX climbMotor = new TalonFX(4);

  private PIDController shooterPivotFeedback = new PIDController(100, 0, 0, 0.02); // Prayer numbers

  private final PositionVoltage positionControl = new PositionVoltage(0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbSupplyCurrent;
  private final StatusSignal<Current> climbTorqueCurrent;
  private final StatusSignal<Temperature> climbTempCelsius;

  public ClimbIOReal() {
    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbSupplyCurrent = climbMotor.getSupplyCurrent();
    climbTorqueCurrent = climbMotor.getTorqueCurrent();
    climbTempCelsius = climbMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        climbPosition,
        climbVelocity,
        climbAppliedVolts,
        climbSupplyCurrent,
        climbTorqueCurrent,
        climbTempCelsius);

    config.Slot0.kP = 4;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicCruiseVelocity = 4;
    config.MotionMagic.MotionMagicAcceleration = 8;
    config.MotionMagic.MotionMagicJerk = 1600;

    climbMotor.getConfigurator().apply(config, 1.0);

    climbMotor.setPosition(0);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.isPositionIOMotorConnected =
        BaseStatusSignal.refreshAll(
                climbPosition,
                climbVelocity,
                climbAppliedVolts,
                climbSupplyCurrent,
                climbTorqueCurrent,
                climbTempCelsius)
            .isOK();
    inputs.positionMechs = climbPosition.getValueAsDouble();
    inputs.velocityRpm = climbVelocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = climbAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = climbSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = climbTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius = climbTempCelsius.getValueAsDouble();
  }

  @Override
  public void setPosition(double pos) // Set the motor position in mechanism rotations
      {
    CatzClimb.position = pos;
    climbMotor.setControl(positionControl.withPosition(pos));
    // System.out.println(pos);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
    climbMotor.getConfigurator().apply(config);
  }

  @Override
  public void runCharacterizationMotor(double input) {
    climbMotor.setControl(voltageControl.withOutput(input));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    System.out.println("kS: " + kS + " kV: " + kV + " kA: " + kA);
    climbMotor.getConfigurator().apply(config);
  }
}
