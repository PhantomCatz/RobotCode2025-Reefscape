// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Utilities;

public class MotorUtil {

  public record Gains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicParameters(
      double mmCruiseVelocity, double mmAcceleration, double mmJerk) {}

  public static enum NeutralMode {
    COAST,
    BREAK
  }
}
