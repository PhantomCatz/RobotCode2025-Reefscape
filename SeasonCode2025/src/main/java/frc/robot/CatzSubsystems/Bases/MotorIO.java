package frc.robot.CatzSubsystems.Bases;

import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;

public interface MotorIO {

  @AutoLog
  public static class MotorIOInputs {

    public boolean isLeaderMotorConnected = false;
    public boolean isFollowerMotorConnected = false;


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

  public default void runPercentOutput(double percent) {}

  public default void setPosition(double pos) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void setNeutralMode(NeutralModeValue mode) {}

  public default void stop() {}

  public default MotorIOInputs getMotorIOInputs() {return new MotorIO.MotorIOInputs();}

  public default void setCoastOut() {}

  public default void setNeutralOut() {}

  public default void setCurrentPosition(Angle mechanismPosition) {}

}
