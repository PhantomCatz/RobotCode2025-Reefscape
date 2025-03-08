// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

  static class ModuleIOInputs {
    public boolean isDriveMotorConnected;
    public double drivePositionUnits;
    public double driveVelocityRPS;
    public double driveAppliedVolts;
    public double driveSupplyCurrentAmps;
    public double driveTorqueCurrentAmps;

    public boolean isSteerMotorConnected;
    public double steerAbsoluteInitPosition;

    public Rotation2d rawAbsEncPosition = new Rotation2d();
    public double rawAbsEncValueRotation;
    public Rotation2d steerAbsPosition = new Rotation2d();

    public double steerTorqueCurrentAmps;
    public double steerSupplyCurrentAmps;
    public double[] odometryDrivePositionsMeters = new double[0];
    public Rotation2d[] odometrySteerPositions = new Rotation2d[0];

    // Simulation Inputs
    public Rotation2d steerPosition = new Rotation2d();
    public double steerVelocityRadsPerSec;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  // ---------------------------------------------------------------------------
  //   Drive Access Methods
  // ---------------------------------------------------------------------------
  public default void runDrivePwrPercentIO(double drivePwrPercent) {}

  public default void runDriveVelocityRPSIO(double velocity) {}

  public default void setDriveNeutralModeIO(NeutralModeValue type) {}

  public default void setDrvSensorPositionIO(double sensorpos) {}

  public default void setDriveSimPwrIO(double volts) {}

  public default void runCharacterization(double input) {}

  public default void setDrivePID(double kP, double kI, double kD) {}

  public default double getDriveVelocity() {
    return 0.0;
  }

  public default double getDriveDistanceUnits() {
    return 0.0;
  }

  // ---------------------------------------------------------------------------
  //   Steer Access Methods
  // ---------------------------------------------------------------------------
  public default void runSteerPercentOutput(double steerPwr) {}

  public default void runSteerPositionSetpoint(double currentAngleRads, double targetAngleRads) {}

  public default void setSteerNeutralModeIO(NeutralModeValue type) {}

  public default void setSteerSimPwrIO(double volts) {}

  public default void setSteerPID(double kP, double kI, double kD) {}

  public default double getSteerEncoder() {
    return 0.0;
  }
  // ---------------------------------------------------------------------------
  //   Mag Enc Access Methods
  // ---------------------------------------------------------------------------
  public default void resetMagEncoderIO() {}
}
