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
import frc.robot.CatzSubsystems.Bases.MotorIOReal;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;
import frc.robot.Utilities.MotorUtil.NeutralMode;

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

    private MotorIOReal rampPivotMtr;

    public RampPivotIOReal() {
        rampPivotMotor = new TalonFX(RAMP_PIVOT_MTR_ID);

        rampPivotMtr = new MotorIOReal(rampPivotMotor, MANUAL_SCALE, gains, motionMagicParameters, NeutralModeValue.Coast);
        // config.CurrentLimits.SupplyCurrentLimit = 100;
        // config.CurrentLimits.SupplyCurrentLimitEnable = true;

        rampPivotPosition = rampPivotMotor.getPosition();
        rampPivotAppliedVolts = rampPivotMotor.getMotorVoltage();
        rampPivotSupplyCurrent = rampPivotMotor.getSupplyCurrent();
        rampPivotTorqueCurrent = rampPivotMotor.getTorqueCurrent();
        rampPivotTempCelsius = rampPivotMotor.getDeviceTemp();
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
    public void runPercentOutput(double percent) {
        rampPivotMtr.runPercentOutput(percent);
    }

    @Override
    public void runMotor(double speed) {
        // System.out.println("pivot motor running at " + speed);
        rampPivotMtr.runMotor(speed);
    }

    @Override
    public void setPosition(double setpointRotations, double feedforward) {
        rampPivotMotor.setControl(positionControl.withPosition(setpointRotations)
                                                 .withFeedForward(feedforward));
    }

    @Override
    public void stop() {
        rampPivotMotor.setControl(new DutyCycleOut(0));
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        if(mode == NeutralMode.BRAKE) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        rampPivotMotor.getConfigurator().apply(config);
    }


}
