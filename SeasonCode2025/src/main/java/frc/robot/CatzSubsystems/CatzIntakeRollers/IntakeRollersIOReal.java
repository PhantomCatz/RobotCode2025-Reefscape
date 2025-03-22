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
package frc.robot.CatzSubsystems.CatzIntakeRollers;

import static frc.robot.CatzSubsystems.CatzIntakeRollers.IntakeRollersConstants.*;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;


public class IntakeRollersIOReal implements IntakeRollersIO{

    private final DigitalInput beamBreakBack;
    private final DigitalInput beamBreakFrnt;

    private final TalonFXS IntakeCoralMtr;
    private final TalonFXSConfiguration config = new TalonFXSConfiguration();

    public IntakeRollersIOReal() {
        IntakeCoralMtr = new TalonFXS(INTAKE_RAMP_ID);

        // config.Slot0.kS = gains.kS();
        // config.Slot0.kV = gains.kV();
        // config.Slot0.kA = gains.kA();
        // config.Slot0.kP = gains.kP();
        // config.Slot0.kI = gains.kI();
        // config.Slot0.kD = gains.kD();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        IntakeCoralMtr.setPosition(0);
        IntakeCoralMtr.getConfigurator().apply(config, 1.0);


        beamBreakBack = new DigitalInput(RAMP_BB_BACK_PORT);
        beamBreakFrnt = new DigitalInput(RAMP_BB_FRNT_PORT);
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        inputs.bbreakRampBackTriggered = !beamBreakBack.get();
        inputs.bbreakRampFrntTriggered = !beamBreakFrnt.get();
    }

    @Override
    public void runIntakeRampMotor(double speed) {
        double volts = speed * 12.0;
      IntakeCoralMtr.setControl(new VoltageOut(volts));
    }

    @Override
    public void adjustIntakeRamp(double pos) {
        IntakeCoralMtr.setPosition(pos);
    }
}
