//------------------------------------------------------------------------------------
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePivotIO {
  @AutoLog
  public static class AlgaePivotIOInputs {
    public boolean isPositionIOMotorConnected = true;

    public double positionDegrees = 0.0;
    public double sparkPosMechs;
    public double velocityRpm = 0.0;
    public double velocityRads = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(AlgaePivotIOInputs inputs) {}

  public default void resetPosition(double pos) {}

  public default void setPercentOutput(double percentOutput) {}

  public default void runSetpointTicks(double setpointTicks) {}

  public default void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default void setVoltage(double volts) {}

  public default void runCharacterizationMotor(double input) {}

  public default void runSetpointUp(double setpointRotations, double feedforward) {}

  public default void runSetpointDown(double setpointRotations, double feedforward) {}


}
