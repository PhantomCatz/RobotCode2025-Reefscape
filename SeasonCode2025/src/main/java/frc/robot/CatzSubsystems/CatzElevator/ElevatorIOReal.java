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

  TalonFX leaderTalon = new TalonFX(LEFT_LEADER_ID);
  TalonFX followerTalon = new TalonFX(RIGHT_LEADER_ID);

  // private final TalonFX leaderTalon;
  // private final TalonFX followerTalon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final MotionMagicVoltage positionControl =
      // new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
      new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

  //Needs to be initialzed TBD
  private final StatusSignal<ControlModeValue> leaderElevatorState;
  private final StatusSignal<Angle> leaderElevatorPosition;
  private final StatusSignal<AngularVelocity> leaderElevatorVelocity;
  private final StatusSignal<Voltage> leaderElevatorAppliedVolts;
  private final StatusSignal<Current> leaderElevatorSupplyCurrent;
  private final StatusSignal<Current> leaderElevatorTorqueCurrent;
  private final StatusSignal<Temperature> leaderElevatorTempCelsius;



  public ElevatorIOReal() {

    leaderElevatorPosition      = leaderTalon.getPosition();
    leaderElevatorVelocity      = leaderTalon.getVelocity();
    leaderElevatorState         = leaderTalon.getControlMode();
    leaderElevatorAppliedVolts  = leaderTalon.getMotorVoltage();
    leaderElevatorSupplyCurrent = leaderTalon.getSupplyCurrent();
    leaderElevatorTorqueCurrent = leaderTalon.getTorqueCurrent();
    leaderElevatorTempCelsius   = leaderTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
      100.0,
      leaderElevatorPosition,
      leaderElevatorVelocity,
      leaderElevatorAppliedVolts,
      leaderElevatorSupplyCurrent,
      leaderElevatorTorqueCurrent,
      leaderElevatorTempCelsius);

    config.Slot0.kP = 12.0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.CurrentLimits.SupplyCurrentLimit = 80.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = 80;
    config.MotionMagic.MotionMagicAcceleration = 400;
    config.MotionMagic.MotionMagicJerk = 1600;

    leaderTalon.getConfigurator().apply(config, 1.0);
    followerTalon.getConfigurator().apply(config, 1.0);

    followerTalon.setControl(new Follower(leaderTalon.getDeviceID(), true));

    leaderTalon.setPosition(0);
    followerTalon.setPosition(0);

  }

    public void updateInputs(ElevatorIOInputs inputs) {
      inputs.isElevatorIOMotorConnected =
        BaseStatusSignal.refreshAll(
          leaderElevatorState,
          leaderElevatorPosition,
          leaderElevatorVelocity,
          leaderElevatorAppliedVolts,
          leaderElevatorSupplyCurrent,
          leaderElevatorTorqueCurrent,
          leaderElevatorTempCelsius)
          .isOK();
      inputs.motorState        = leaderElevatorState.getValueAsDouble();
      inputs.leaderPositionRotations = leaderElevatorPosition.getValueAsDouble();
      inputs.velocityRpm       = leaderElevatorVelocity.getValueAsDouble() * 60.0;
      inputs.appliedVolts      = leaderElevatorAppliedVolts.getValueAsDouble();
      inputs.supplyCurrentAmps = leaderElevatorSupplyCurrent.getValueAsDouble();
      inputs.torqueCurrentAmps = leaderElevatorTorqueCurrent.getValueAsDouble();
      inputs.tempCelsius       = leaderElevatorTempCelsius.getValueAsDouble();
    }


  @Override
  public void runSetpoint(double setpointRotations, double feedforward) {
    // System.out.println(setpointRotations);
    leaderTalon.setControl(positionControl.withPosition(setpointRotations).withFeedForward(feedforward));
  }

  @Override
  public void setPosition(double pos) {
    CatzElevator.position = pos;
    leaderTalon.setControl(positionControl.withPosition(pos));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
    leaderTalon.getConfigurator().apply(config);
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

  @Override
  public void setFF(double kS, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    System.out.println("kS: " + kS + " kV: " + kV + " kA: " + kA);
    leaderTalon.getConfigurator().apply(config);
  }

}
