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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;



public class ElevatorIOReal implements ElevatorIO {
  // Motor Instantiation
  TalonFX leaderTalon = new TalonFX(RIGHT_FOLLOWER_ID);

  // Motor configuration
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final MotionMagicVoltage positionControlUp = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
  private final MotionMagicVoltage positionControlDown = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

  // Status Signals
  private final StatusSignal<Angle> internalPositionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // External Sensors
  final DigitalInput m_elevatorLimitBot = new DigitalInput(BOT_LIMIT_SWITCH);

  public ElevatorIOReal() {

    // Status signals
    internalPositionRotations = leaderTalon.getPosition();
    velocityRps = leaderTalon.getVelocity();
    appliedVoltage = leaderTalon.getMotorVoltage();
    supplyCurrent = leaderTalon.getSupplyCurrent();
    torqueCurrent = leaderTalon.getTorqueCurrent();
    tempCelsius = leaderTalon.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius
    );

    // PID configs
    config.Slot0.kS = slot0_gains.kS();
    config.Slot0.kV = slot0_gains.kV();
    config.Slot0.kA = slot0_gains.kA();
    config.Slot0.kP = slot0_gains.kP();
    config.Slot0.kI = slot0_gains.kI();
    config.Slot0.kD = slot0_gains.kD();
    config.Slot0.kG = slot0_gains.kG();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.Slot1.kS = slot1_gains.kS();
    config.Slot1.kV = slot1_gains.kV();
    config.Slot1.kA = slot1_gains.kA();
    config.Slot1.kP = slot1_gains.kP();
    config.Slot1.kI = slot1_gains.kI();
    config.Slot1.kD = slot1_gains.kD();
    config.Slot1.kG = slot1_gains.kG();
    config.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    // Motion Magic Parameters
    config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
    config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
    config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();

    // Encoder Inversion
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Current Limits
    config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.CurrentLimits.SupplyCurrentLimit = 80.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Encoder Resetting
    leaderTalon.setPosition(0);

    // Applying Configs
    leaderTalon.getConfigurator().apply(config, 1.0);

    // Setting follower
    positionControlUp.EnableFOC = true;
    positionControlUp.Slot = 0;

    positionControlDown.EnableFOC = true;
    positionControlDown.Slot = 1;
  }

    public void updateInputs(ElevatorIOInputs inputs) {

      inputs.isLeaderMotorConnected =
          BaseStatusSignal.refreshAll(
                  internalPositionRotations,
                  velocityRps,
                  appliedVoltage,
                  supplyCurrent,
                  torqueCurrent,
                  tempCelsius)
              .isOK();

      inputs.positionInch =       internalPositionRotations.getValueAsDouble() * FINAL_RATIO;
      inputs.velocityInchPerSec = velocityRps.getValueAsDouble() * FINAL_RATIO;
      inputs.appliedVolts =       appliedVoltage.getValueAsDouble();
      inputs.supplyCurrentAmps =  supplyCurrent.getValueAsDouble();
      inputs.torqueCurrentAmps =  torqueCurrent.getValueAsDouble();
      inputs.tempCelcius =        tempCelsius.getValueAsDouble();

      inputs.isBotLimitSwitched = m_elevatorLimitBot.get();
    }


  @Override
  public void runSetpointUp(double setpointInches) {
    double setpointRotations = setpointInches/FINAL_RATIO;
    leaderTalon.setControl(positionControlUp.withPosition(setpointRotations));
  }

  @Override
  public void runSetpointDown(double setpointInches) {
    double setpointRotations = setpointInches/FINAL_RATIO;
    leaderTalon.setControl(positionControlDown.withPosition(setpointRotations));
  }

  @Override
  public void stop() {
    leaderTalon.setControl(new DutyCycleOut(0.0));
  }

  @Override
  public void resetPosition(double pos) {
    leaderTalon.setPosition(pos);
  }

  @Override
  public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kG = kG;
    leaderTalon.getConfigurator().apply(config);
  }

  @Override
  public void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    config.Slot1.kP = kP;
    config.Slot1.kI = kI;
    config.Slot1.kD = kD;
    config.Slot1.kS = kS;
    config.Slot1.kV = kV;
    config.Slot1.kA = kA;
    config.Slot1.kG = kG;
    leaderTalon.getConfigurator().apply(config);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void runMotor(double speed) {
    System.out.println(speed);
    leaderTalon.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    leaderTalon.getConfigurator().apply(config);
  }

}
