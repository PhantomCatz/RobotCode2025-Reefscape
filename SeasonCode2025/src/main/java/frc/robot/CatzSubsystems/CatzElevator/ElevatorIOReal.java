// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;


public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final MotionMagicVoltage positionControl =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

  //Needs to be initialzed TBD
  // private final StatusSignal<ControlModeValue> motorState;
  // private final StatusSignal<Double> internalPositionRotations;
  // private final StatusSignal<Double> velocityRPM;
  // private final StatusSignal<Double> appliedVoltage;
  // private final StatusSignal<Double> supplyCurrent;
  // private final StatusSignal<Double> torqueCurrent;
  // private final StatusSignal<Double> tempCelsius;

  public ElevatorIOReal() {
    leaderTalon = new TalonFX(leaderID);
    followerTalon = new TalonFX(followerID);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderTalon.getConfigurator().apply(config, 1.0);
    followerTalon.getConfigurator().apply(config, 1.0);

    // motorState = leaderTalon.getControlMode();
    // velocityRPM = leaderTalon.getVelocity();
    // internalPositionRotations = leaderTalon.getPosition();

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
