// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzRampPivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.CatzSubsystems.CatzRampPivot.RampPivotConstants.*;


/** Add your docs here. */
public class RampPivotIOReal implements RampPivotIO{

    private final TalonFX rampPivotMotor;
    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

    private final StatusSignal<Voltage> rampPivotAppliedVolts;
    private final StatusSignal<Current> rampPivotSupplyCurrent;
    private final StatusSignal<Current> rampPivotTorqueCurrent;
    private final StatusSignal<Temperature> rampPivotTempCelsius;
    private final StatusSignal<Angle> rampPivotPosition;

    public RampPivotIOReal() {
        rampPivotMotor = new TalonFX(RAMP_PIVOT_MTR_ID);

        config.CurrentLimits.SupplyCurrentLimit = 100;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rampPivotPosition = rampPivotMotor.getPosition();
        rampPivotAppliedVolts = rampPivotMotor.getMotorVoltage();
        rampPivotSupplyCurrent = rampPivotMotor.getSupplyCurrent();
        rampPivotTorqueCurrent = rampPivotMotor.getTorqueCurrent();
        rampPivotTempCelsius = rampPivotMotor.getDeviceTemp();

        // PID configs
        config.Slot0.kS = gains.kS();
        config.Slot0.kV = gains.kV();
        config.Slot0.kA = gains.kA();
        config.Slot0.kP = gains.kP();
        config.Slot0.kI = gains.kI();
        config.Slot0.kD = gains.kD();

        config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
        config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
        config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();

        rampPivotMotor.getConfigurator().apply(config, 1.0);

        rampPivotMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(RampPivotIOInputs inputs) {
        inputs.isRampPivotMotorConnected =
            BaseStatusSignal.refreshAll(
                    rampPivotPosition,
                    rampPivotAppliedVolts,
                    rampPivotSupplyCurrent,
                    rampPivotTorqueCurrent,
                    rampPivotTempCelsius)
                .isOK();
        inputs.positionMechs =     rampPivotPosition.getValueAsDouble();
        inputs.appliedVolts      = rampPivotAppliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = rampPivotSupplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = rampPivotTorqueCurrent.getValueAsDouble();
        inputs.tempCelsius       = rampPivotTempCelsius.getValueAsDouble();
    }

    @Override
    public void runPercentOutput(double speed) {
        rampPivotMotor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void runMotor(double speed) {
        System.out.println("i am running - pivot motor");
        rampPivotMotor.set(speed);
    }

    @Override
    public void setPosition(double setpointRotations, double feedforward) {
        System.out.println(setpointRotations);
        rampPivotMotor.setControl(positionControl.withPosition(setpointRotations)
                                                 .withFeedForward(feedforward));
    }

    @Override
    public void stop() {
        rampPivotMotor.setControl(new DutyCycleOut(0));
    }
}
