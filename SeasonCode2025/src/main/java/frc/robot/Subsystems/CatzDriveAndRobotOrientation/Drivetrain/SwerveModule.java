// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Subsystems.CatzDriveAndRobotOrientation.Drivetrain;

import static frc.robot.Subsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.Subsystems.CatzDriveAndRobotOrientation.Drivetrain.ModuleIO.ModuleIOInputs;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.CatzMathUtils;
import frc.robot.Utilities.CatzMathUtils.Conversions;

public class SwerveModule {

  // Module delcaration block
  private final ModuleIO io;
  private final ModuleIOInputs inputs = new ModuleIOInputs();

  // Module Strings for Logging
  private final String m_moduleName;

  // Global swerve module variables
  private SwerveModuleState m_swerveModuleState = new SwerveModuleState();

  // Alerts
  private final Alert driveMotorDisconnected;
  private final Alert steerMotorDisconnected;

  // ----------------------------------------------------------------------------------------------
  //
  //  CatzServeModule() - Constructor
  //
  // ----------------------------------------------------------------------------------------------
  public SwerveModule(ModuleIDs config, String moduleName) {
    this.m_moduleName = moduleName;
    // Run Subsystem disconnect check
    if (DriveConstants.IS_DRIVE_DISABLED) {
      io = new ModuleIONull();
      System.out.println("Module " + m_moduleName + " Unconfigured");
    } else {
      // Run Robot Mode hardware assignment
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new ModuleIORealFoc(config, m_moduleName);
          System.out.println("Module " + m_moduleName + " Configured for Real");
          break;
        case REPLAY:
          io = new ModuleIORealFoc(config, m_moduleName) {};
          System.out.println("Module " + m_moduleName + " Configured for Replay simulation");
          break;
        case SIM:
          io = new ModuleIOSim(config);
          System.out.println("Module " + m_moduleName + " Configured for WPILIB simulation");
          break;
        default:
          io = null;
          System.out.println("Module " + m_moduleName + " Unconfigured");
          break;
      }
    }

    // Disconnected Alerts
    driveMotorDisconnected =
        new Alert(m_moduleName + " drive motor disconnected!", Alert.AlertType.kError);
    steerMotorDisconnected =
        new Alert(m_moduleName + " steer motor disconnected!", Alert.AlertType.kError);

    resetDriveEncs();
  } // -End of CatzSwerveModule Constructor

  // ----------------------------------------------------------------------------------------------
  //
  //  Periodic
  //
  // ----------------------------------------------------------------------------------------------
  public void periodic() {
    // Process and Log Module Inputs
    io.updateInputs(inputs);

    debugLogsSwerve();
  } // -End of CatzSwerveModule Periodic

  public void debugLogsSwerve() {

    // Logger.recordOutput("Module " + m_moduleName + "/targetmoduleangle rad",
    // m_swerveModuleState.angle.getRadians());

    // SmartDashboard.putNumber("absencposrad" + m_moduleName,
    // inputs.steerAbsoluteEncPosition.getRadians());
    SmartDashboard.putNumber("angle" + m_moduleName, getCurrentRotation().getDegrees());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setModuleAngleAndVelocity(SwerveModuleState state) {
    // --------------------------------------------------------
    // Collect variabels used in calculations
    // --------------------------------------------------------
    this.m_swerveModuleState = state;
    double targetAngleRads = state.angle.getRadians();
    double currentAngleRads = getAbsEncRadians();
    if(m_moduleName == DriveConstants.MODULE_NAMES[1]) {

    }
    // --------------------------------------------------------
    // Run closed loop drive control
    // --------------------------------------------------------
    io.runDriveVelocity(state.speedMetersPerSecond);
    // --------------------------------------------------------
    // Run closed loop steer control
    // --------------------------------------------------------
    io.runSteerPositionSetpoint(currentAngleRads, targetAngleRads);
  }

  // --------------------------------------------------------------------------------------------------------------------
  //
  //  Drivetrain Power Setting methods
  //
  // --------------------------------------------------------------------------------------------------------------------
  public void setSteerPower(double pwr) {
    io.runSteerPercentOutput(pwr);
  }

  public void setDriveVelocity(double velocity) {
    io.runDriveVelocity(velocity);
  }

  public void stopDriving() {
    io.runDriveVelocity(0.0);
  }

  // --------------------------------------------------------------------------------------------------------------------
  //
  //  Module Util Methods
  //
  // --------------------------------------------------------------------------------------------------------------------
  public void setNeutralModeDrive(NeutralModeValue type) {
    io.setDriveNeutralModeIO(type);
  }

  public void setNeutralModeSteer(NeutralModeValue type) {
    io.setSteerNeutralModeIO(type);
  }

  public void resetDriveEncs() {
    io.setDrvSensorPositionIO(0.0);
  }

  /** optimze wheel angles before sending to setdesiredstate method for logging */
  public SwerveModuleState optimizeWheelAngles(SwerveModuleState unoptimizedState) {
    SwerveModuleState optimizedState =
        CatzMathUtils.optimize(unoptimizedState, getCurrentRotation());
    return optimizedState;
  }

  // --------------------------------------------------------------------------------------------------------------------
  //
  //  Module getters
  //
  // --------------------------------------------------------------------------------------------------------------------
  public SwerveModuleState getModuleState() {
    double velocityMPS = CatzMathUtils.Conversions.RPSToMPS(io.getDriveVelocity());

    return new SwerveModuleState(velocityMPS, getCurrentRotation());
  }

  public SwerveModuleState getModuleStateSetpoint() {
    return m_swerveModuleState;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
  }

  public double getDriveDistanceMeters() {
    // seconds cancels out
    return CatzMathUtils.Conversions.RPSToMPS(io.getDriveDistanceUnits());
  }


  /** Get velocity of drive wheel for characterization */
  public double getCharacterizationVelocityRadPerSec() {
    return Units.rotationsToRadians(getDrvVelocityRPS());
  }

  public double getDrvVelocityRPS() {
    return io.getDriveVelocity();
  }

  /** Outputs the Rotation object of the module */
  public Rotation2d getCurrentRotation() {
    return new Rotation2d(getAbsEncRadians());
  }

  private double getAbsEncRadians() {
    // mag enc value should already have offset applied
    return Units.rotationsToRadians(io.getSteerEncoder());
  }
}
