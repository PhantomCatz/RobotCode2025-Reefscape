// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzElevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX elevatorMotor1;
  private final TalonFX elevatorMotor2;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final MotionMagicVoltage positionControl =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

  public ElevatorIOReal() {
    elevatorMotor1 = new TalonFX(0);
    elevatorMotor2 = new TalonFX(1);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor1.getConfigurator().apply(config, 1.0);
    elevatorMotor2.getConfigurator().apply(config, 1.0);
  }

  @Override
  public void runSetpoint(double setpointRotations, double feedforward) {
    // System.out.println(setpointRotations);
    elevatorMotor1.setControl(
        positionControl.withPosition(setpointRotations).withFeedForward(feedforward));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    elevatorMotor1.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    elevatorMotor2.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void runMotor(double speed) {
    elevatorMotor1.set(speed);
    elevatorMotor2.set(-speed);
  }
}
