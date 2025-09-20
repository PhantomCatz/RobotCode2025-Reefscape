package frc.robot.CatzSubsystems.Bases;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  @AutoLog
  public static class MotorIOInputs {

    public boolean isLeaderMotorConnected = false;
    public boolean isFollowerMotorConnected = false;
    public boolean isBotLimitSwitched1 = false;
    public boolean isBotLimitSwitched2 = false;
    public boolean bbreak1Triggered;
    public boolean bbreak2Triggered;

    public double positionInch = 0.0;
    public double absoluteEncoderPositionRads = 0.0;
    public double relativeEncoderPositionRads = 0.0;
    public double velocityInchPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

  }

  public default void updateInputs(MotorIOInputs inputs) {}

  public default void runMotor(double Speed) {}

  public default void runMotorBck(double Speed) {}

  public default void runCurrent(double amps) {}

  public default void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void runCharacterizationMotor(double input) {}

  public default void runSetpointFrwd(double setpointInches) {}

  public default void runSetpointBack(double setpointInches) {}

  public default void setPosition(double pos) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}

  public default void stop() {

  }

}
