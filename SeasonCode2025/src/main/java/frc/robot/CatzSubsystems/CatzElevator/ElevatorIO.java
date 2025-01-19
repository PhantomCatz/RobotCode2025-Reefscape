// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzElevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {

    public boolean isElevatorIOMotorConnected = true;

    public double positionMechs = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void runMotor(double Speed) {}

  public default void runMotorBck(double Speed) {}

  public default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void runCharacterizationMotor(double input) {}

  public default void runSetpoint(double setpointRotations, double feedforward) {}

  public default void setBrakeMode(boolean enabled) {}
}
