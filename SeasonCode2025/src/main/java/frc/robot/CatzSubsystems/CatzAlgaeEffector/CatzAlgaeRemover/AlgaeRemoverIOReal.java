// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.AlgaeRemoverConstants.*;


public class AlgaeRemoverIOReal implements AlgaeRemoverIO {

  private final TalonFXS algaeRemoverMotor;
  private final TalonFXSConfiguration config = new TalonFXSConfiguration();

  private final StatusSignal<Voltage> algaeRemoverAppliedVolts;
  private final StatusSignal<Current> algaeRemoverSupplyCurrent;
  private final StatusSignal<Current> algaeRemoverTorqueCurrent;
  private final StatusSignal<Temperature> algaeRemoverTempCelsius;
  private final StatusSignal<Angle> algaeRemoverPosition;

  public AlgaeRemoverIOReal() {
    algaeRemoverMotor = new TalonFXS(ALGAE_REMOVER_MOTOR_ID);

    config.CurrentLimits.SupplyCurrentLimit = STALL_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    algaeRemoverPosition = algaeRemoverMotor.getPosition();
    algaeRemoverAppliedVolts = algaeRemoverMotor.getMotorVoltage();
    algaeRemoverSupplyCurrent = algaeRemoverMotor.getSupplyCurrent();
    algaeRemoverTorqueCurrent = algaeRemoverMotor.getTorqueCurrent();
    algaeRemoverTempCelsius = algaeRemoverMotor.getDeviceTemp();

    algaeRemoverMotor.getConfigurator().apply(config, 1.0);
  }

  @Override
  public void updateInputs(AlgaeEffectorIOInputs inputs) {
    inputs.isAlgaeEffectorMotorConnected =
        BaseStatusSignal.refreshAll(
                algaeRemoverPosition,
                algaeRemoverAppliedVolts,
                algaeRemoverSupplyCurrent,
                algaeRemoverTorqueCurrent,
                algaeRemoverTempCelsius)
            .isOK();
    inputs.positionMechs =     algaeRemoverPosition.getValueAsDouble();
    inputs.appliedVolts      = algaeRemoverAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = algaeRemoverSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = algaeRemoverTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius       = algaeRemoverTempCelsius.getValueAsDouble();
  }

  @Override
  public void runPercentOutput(double speed) {
    System.out.println(speed + " ");
    algaeRemoverMotor.setControl(new DutyCycleOut(speed));
  }
}
