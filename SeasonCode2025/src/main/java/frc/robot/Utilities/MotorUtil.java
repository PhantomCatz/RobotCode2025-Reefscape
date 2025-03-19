//------------------------------------------------------------------------------------
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.Utilities;

public class MotorUtil {

  public record Gains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicParameters(
      double mmCruiseVelocity, double mmAcceleration, double mmJerk) {}

  public static enum NeutralMode {
    COAST,
    BRAKE
  }
}
