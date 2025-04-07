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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.AlgaePivotConstants.*;


public class AlgaePivotIONull implements AlgaePivotIO{


    private TalonFX algaePivotMotor = new TalonFX(ALGAE_PIVOT_MOTOR_ID);


  private final TalonFXConfiguration config = new TalonFXConfiguration();

    AlgaePivotIONull() {
      config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      algaePivotMotor.getConfigurator().apply(config, 1.0);

      algaePivotMotor.setPosition(PIVOT_INITIAL_POS);
    }
}
