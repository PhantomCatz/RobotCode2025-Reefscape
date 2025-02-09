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


  TalonFX leaderTalon = new TalonFX(LEFT_LEADER_ID);
  TalonFX followerTalon = new TalonFX(RIGHT_FOLLOWER_ID);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

  // Status Signals
  private final StatusSignal<Angle> internalPositionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final List<StatusSignal<Voltage>> appliedVoltage;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> tempCelsius;

  final DigitalInput m_elevatorLimitTop = new DigitalInput(TOP_LIMIT_SWITCH);
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

    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kA = gains.kA();
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();

    config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
    config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
    config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();

    leaderTalon.getConfigurator().apply(config, 1.0);
    followerTalon.getConfigurator().apply(config, 1.0);

    followerTalon.setControl(new Follower(leaderTalon.getDeviceID(), true));
    positionControl.EnableFOC = true;

    leaderTalon.setPosition(0);
    followerTalon.setPosition(0);


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

      inputs.positionRads = Units.rotationsToRadians(internalPositionRotations.getValueAsDouble());
      inputs.velocityRadsPerSec = Units.rotationsToRadians(velocityRps.getValueAsDouble());
      inputs.appliedVolts =
          appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
      inputs.supplyCurrentAmps =
          supplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
      inputs.torqueCurrentAmps =
          torqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
      inputs.tempCelcius = tempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();

      inputs.isTopLimitSwitched = m_elevatorLimitTop.get();
      inputs.isBotLimitSwitched = m_elevatorLimitBot.get();
    }


  @Override
  public void runSetpoint(double setpointRotations, double feedforward) {
    // System.out.println(setpointRotations);
    leaderTalon.setControl(positionControl.withPosition(setpointRotations)
                                          .withFeedForward(feedforward)
                                          .withLimitForwardMotion(m_elevatorLimitTop.get())
                                          .withLimitReverseMotion(m_elevatorLimitBot.get()));
  }

  @Override
  public void setPosition(double pos) {
    CatzElevator.position = pos;
    leaderTalon.setPosition(pos);
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
