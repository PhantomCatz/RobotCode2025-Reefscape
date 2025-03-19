//------------------------------------------------------------------------------------
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public double gyroAngle = 0.0;
    public double gyroYawDegrees = 0.0;
    public double gyroRollDegrees = 0.0;
    public double gyroPitch = 0.0;
    public boolean gyroConnected = false;
    public double gyroYawVel = 0.0;
    public double gyroAccelX = 0.0;
    public double gyroAccelY = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
