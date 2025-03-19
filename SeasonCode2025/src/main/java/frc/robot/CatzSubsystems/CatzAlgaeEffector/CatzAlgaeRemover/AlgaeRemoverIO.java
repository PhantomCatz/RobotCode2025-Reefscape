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
package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRemoverIO {

  @AutoLog
  public static class AlgaeEffectorIOInputs {

    public boolean isAlgaeEffectorMotorConnected = true;

    public double positionMechs = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public boolean bbreakTriggered;
  }

  public default void updateInputs(AlgaeEffectorIOInputs inputs) {}

  public default void runPercentOutput(double Speed) {}

  public default void runPercentOutputBck(double Speed) {}

  public default void runSparkMax(double Speed) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void runCharacterizationMotor(double input) {}

}
