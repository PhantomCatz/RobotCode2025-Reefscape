// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzOuttake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {

    public boolean isIntakeIOMotorConnected = true;

    public double positionMechs = 0.0;
    public double velocityRpmLeft = 0.0;
    public double velocityRpmRight = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public boolean bbreakFrntTriggered;
    public boolean bbreakBackTriggered;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void runIntakesIntakeMotor(double speed) {}

  public default void runMotor(double Speed, double speed2) {}

  public default void setPIDF(double kP, double kI, double kD, double kF) {}
}
