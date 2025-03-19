//------------------------------------------------------------------------------------
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems.CatzClimb;


import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public boolean isPositionIOMotorConnected = true;

    public double positionMechs = 0.0;
    public double sparkPosMechs;
    public double velocityRpm = 0.0;
    public double velocityRads = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setPosition(double pos) {}

  public default void runSetpointTicks(double setpointTicks) {}

  public default void setPID(double kP, double kI, double kD) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void runCharacterizationMotor(double input) {}

  //   public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setPower(double joystickPower) {}

}
