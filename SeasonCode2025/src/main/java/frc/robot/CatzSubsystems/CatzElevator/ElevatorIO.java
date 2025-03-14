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

    public boolean isLeaderMotorConnected = false;
    public boolean isFollowerMotorConnected = false;
    public boolean isBotLimitSwitched = false;

    public double positionRads = 0.0;
    public double absoluteEncoderPositionRads = 0.0;
    public double relativeEncoderPositionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runMotor(double Speed) {}

  public default void runMotorBck(double Speed) {}

  public default void runCurrent(double amps) {}

  public default void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void runCharacterizationMotor(double input) {}

  public default void runSetpointUp(double setpointRotations) {}

  public default void runSetpointDown(double setpointRotations) {}

  public default void resetPosition(double pos) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}

  public default void stop() {

  }
}
