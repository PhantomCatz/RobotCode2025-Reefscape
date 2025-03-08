// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;


public interface GyroIO {
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

  public default double getAngle() {
    return 0.0;
  }
}
