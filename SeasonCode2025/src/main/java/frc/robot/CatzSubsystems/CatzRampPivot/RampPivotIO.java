//------------------------------------------------------------------------------------
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems.CatzRampPivot;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Utilities.MotorUtil.NeutralMode;

public interface RampPivotIO {

    @AutoLog
    public static class RampPivotIOInputs {

      public boolean isRampPivotMotorConnected = true;

      public double positionMechs = 0.0;
      public double velocityRpm = 0.0;
      public double appliedVolts = 0.0;
      public double supplyCurrentAmps = 0.0;
      public double torqueCurrentAmps = 0.0;
      public double tempCelsius = 0.0;
    }

    public default void updateInputs(RampPivotIOInputs inputs) {}

    public default void runPercentOutput(double speed) {}

    public default void setPosition(double setpointRads, double feedforward) {}

    public default void runMotor(double speed) {}

    public default void stop() {}

    public default void setNeutralMode(NeutralMode mode) {}
  }
