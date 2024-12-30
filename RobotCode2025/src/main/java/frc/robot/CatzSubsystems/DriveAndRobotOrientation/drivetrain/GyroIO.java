package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
    public double gyroAngle;
    public double gyroYawDegrees;
    public double gyroRollDegrees;
    public double gyroPitch;
    public boolean gyroConnected;
    public double gyroYawVel;
    public double gyroAccelX;
    public double gyroAccelY;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

}
