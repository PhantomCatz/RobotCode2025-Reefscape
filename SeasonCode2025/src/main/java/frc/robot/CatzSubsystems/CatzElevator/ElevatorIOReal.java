// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import java.util.List;

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
  TalonFX leaderTalon = new TalonFX(LEFT_LEADER_ID);
  TalonFX followerTalon = new TalonFX(RIGHT_FOLLOWER_ID);

  // Motor configuration
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

  // Status Signals
  private final StatusSignal<Angle> internalPositionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final List<StatusSignal<Voltage>> appliedVoltage;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> tempCelsius;

  // External Sensors
  final DigitalInput m_elevatorLimitBot = new DigitalInput(BOT_LIMIT_SWITCH);

  public ElevatorIOReal() {

    // Status signals
    internalPositionRotations = leaderTalon.getPosition();
    velocityRps = leaderTalon.getVelocity();
    appliedVoltage = List.of(leaderTalon.getMotorVoltage(), followerTalon.getMotorVoltage());
    supplyCurrent = List.of(leaderTalon.getSupplyCurrent(), followerTalon.getSupplyCurrent());
    torqueCurrent = List.of(leaderTalon.getTorqueCurrent(), followerTalon.getTorqueCurrent());
    tempCelsius = List.of(leaderTalon.getDeviceTemp(), followerTalon.getDeviceTemp());
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        tempCelsius.get(0),
        tempCelsius.get(1));

    // PID configs
    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kA = gains.kA();
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();

    // Supply Current Limits
    config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Motion Magic Parameters
    config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
    config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
    config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Encoder Resetting
    leaderTalon.setPosition(0);
    followerTalon.setPosition(0);

    // Applying Configs
    leaderTalon.getConfigurator().apply(config, 1.0);
    followerTalon.getConfigurator().apply(config, 1.0);

    // Setting follower
    followerTalon.setControl(new Follower(leaderTalon.getDeviceID(), true));
    positionControl.EnableFOC = true;
  }

    public void updateInputs(ElevatorIOInputs inputs) {

      inputs.isLeaderMotorConnected =
          BaseStatusSignal.refreshAll(
                  internalPositionRotations,
                  velocityRps,
                  appliedVoltage.get(0),
                  supplyCurrent.get(0),
                  torqueCurrent.get(0),
                  tempCelsius.get(0))
              .isOK();

      inputs.isFollowerMotorConnected =
          BaseStatusSignal.refreshAll(
                  appliedVoltage.get(1),
                  supplyCurrent.get(1),
                  torqueCurrent.get(1),
                  tempCelsius.get(1))
              .isOK();

      inputs.positionRads =       Units.rotationsToRadians(internalPositionRotations.getValueAsDouble());
      inputs.velocityRadsPerSec = Units.rotationsToRadians(velocityRps.getValueAsDouble());
      inputs.appliedVolts =       appliedVoltage.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
      inputs.supplyCurrentAmps =  supplyCurrent.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
      inputs.torqueCurrentAmps =  torqueCurrent.stream()
                                              .mapToDouble(StatusSignal::getValueAsDouble)
                                              .toArray();
      inputs.tempCelcius =        tempCelsius.stream()
                                            .mapToDouble(StatusSignal::getValueAsDouble)
                                            .toArray();

      inputs.isBotLimitSwitched = m_elevatorLimitBot.get();
    }


  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    double setpointRotations = Units.radiansToRotations(setpointRads);
    leaderTalon.setControl(positionControl.withPosition(setpointRotations)
                                          .withFeedForward(feedforward + Math.max((setpointRads - 76.2) / 60 * 0.4, 0)));
  }

  @Override
  public void stop() {
    leaderTalon.setControl(new DutyCycleOut(0.0));
  }

  @Override
  public void setPosition(double pos) {
    leaderTalon.setPosition(pos);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    leaderTalon.getConfigurator().apply(config);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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
