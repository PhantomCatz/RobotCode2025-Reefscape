// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final MotionMagicVoltage positionControl =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

  //Needs to be initialzed TBD
  private final StatusSignal<ControlModeValue> motorState;
  private final StatusSignal<Angle> internalPositionRotations;
  private final StatusSignal<AngularVelocity> velocityRPM;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public ElevatorIOReal() {
    leaderTalon = new TalonFX(leaderID);
    followerTalon = new TalonFX(followerID);
    config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderTalon.getConfigurator().apply(config, 1.0);
    followerTalon.getConfigurator().apply(config, 1.0);
    

    motorState = leaderTalon.getControlMode();
    velocityRPM = leaderTalon.getVelocity();
    internalPositionRotations = leaderTalon.getPosition();
    appliedVoltage = leaderTalon.getMotorVoltage();
    supplyCurrent = leaderTalon.getSupplyCurrent();
    torqueCurrent = leaderTalon.getTorqueCurrent();
    tempCelsius = leaderTalon.getDeviceTemp();

  }

    public void updateInputs(ElevatorIOInputs inputs) {
      inputs.isElevatorIOMotorConnected =
        BaseStatusSignal.refreshAll(
          motorState,
          internalPositionRotations,
          velocityRPM,
          appliedVoltage,
          supplyCurrent,
          torqueCurrent,
          tempCelsius)
          .isOK();
      inputs.motorState        = motorState.getValueAsDouble();
      inputs.leaderPositionRotations = internalPositionRotations.getValueAsDouble();
      inputs.velocityRpm       = velocityRPM.getValueAsDouble();
      inputs.appliedVolts      = appliedVoltage.getValueAsDouble();
      inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
      inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
      inputs.tempCelsius       = tempCelsius.getValueAsDouble();
    }


  @Override
  public void runSetpoint(double setpointRotations, double feedforward) {
    System.out.println(setpointRotations);
    leaderTalon.setControl(positionControl.withPosition(setpointRotations).withFeedForward(feedforward));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void runMotor(double speed) {
    leaderTalon.set(speed);
    followerTalon.set(-speed);
  }
}
