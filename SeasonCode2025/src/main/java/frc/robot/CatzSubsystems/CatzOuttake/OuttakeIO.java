//------------------------------------------------------------------------------------
// 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project. 
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
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
    public double leftCurrentAmps = 0.0;
    public double rightCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public boolean bbreakFrntTriggered;
    public boolean bbreakBackTriggered;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void runMotor(double Speed, double speed2) {}

  public default void setPIDF(double kP, double kI, double kD, double kF) {}
}
