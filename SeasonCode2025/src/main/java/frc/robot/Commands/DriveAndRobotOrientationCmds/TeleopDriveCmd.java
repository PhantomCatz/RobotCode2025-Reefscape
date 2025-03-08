// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.Subsystems.CatzDriveAndRobotOrientation.RobotTracker;
import frc.robot.Subsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Subsystems.CatzDriveAndRobotOrientation.Drivetrain.driverain;
import frc.robot.Utilities.AllianceFlipUtil;
import java.util.function.Supplier;

/**************************************************************************************************
 *
 * TeleopDriveCmd
 *
 **************************************************************************************************/

public class TeleopDriveCmd extends Command {
  // Subsystem declaration
  private final driverain m_drivetrain;

  // Xbox controller buttons
  private final Supplier<Double> m_headingPctOutput_X;
  private final Supplier<Double> m_headingPctOutput_Y;
  private final Supplier<Double> m_angVelocityPctOutput;

  // drive variables
  private double m_headingAndVelocity_X;
  private double m_headingAndVelocity_Y;
  private double turningVelocity;

  private ChassisSpeeds chassisSpeeds;

  // --------------------------------------------------------------------------------------
  //
  // Teleop Drive Command Constructor
  //
  // --------------------------------------------------------------------------------------
  public TeleopDriveCmd(
      Supplier<Double> supplierLeftJoyX,
      Supplier<Double> supplierLeftJoyY,
      Supplier<Double> angVelocityPctOutput,
      driverain drivetrain) {
    // Chassis magnatude and direction control
    this.m_headingPctOutput_X = supplierLeftJoyX;
    this.m_headingPctOutput_Y = supplierLeftJoyY;
    this.m_angVelocityPctOutput = angVelocityPctOutput;

    // subsystem assignment
    this.m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
  }

  // --------------------------------------------------------------------------------------
  //
  // Initialize
  //
  // --------------------------------------------------------------------------------------
  @Override
  public void initialize() {}

  // --------------------------------------------------------------------------------------
  //
  // Execute
  //
  // --------------------------------------------------------------------------------------
  @Override
  public void execute() {
    // Obtain realtime joystick inputs with supplier methods
    m_headingAndVelocity_X = -m_headingPctOutput_Y.get(); //Raw accel
    m_headingAndVelocity_Y = -m_headingPctOutput_X.get();
    turningVelocity        = -m_angVelocityPctOutput.get(); // alliance flip shouldn't change for turing speed when switching alliances

    // Flip Directions for left joystick if alliance is red
    if (AllianceFlipUtil.shouldFlipToRed()) {
      m_headingAndVelocity_X = -m_headingAndVelocity_X;
      m_headingAndVelocity_Y = -m_headingAndVelocity_Y;
    }

    // Apply deadbands to prevent modules from receiving unintentional pwr due to joysticks having offset
    m_headingAndVelocity_X =
        Math.abs(m_headingAndVelocity_X) > XboxInterfaceConstants.kDeadband
            ? m_headingAndVelocity_X * DriveConstants.DRIVE_CONFIG.maxLinearVelocity()
            : 0.0;
    m_headingAndVelocity_Y =
        Math.abs(m_headingAndVelocity_Y) > XboxInterfaceConstants.kDeadband
            ? m_headingAndVelocity_Y * DriveConstants.DRIVE_CONFIG.maxLinearVelocity()
            : 0.0;
    turningVelocity =
        Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband
            ? turningVelocity * DriveConstants.DRIVE_CONFIG.maxAngularVelocity()
            : 0.0;

    // Construct desired chassis speeds
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_headingAndVelocity_X,
                                                          m_headingAndVelocity_Y,
                                                          turningVelocity,
                                                          RobotTracker.getInstance().getEstimatedPose().getRotation());

    // Send new chassisspeeds object to the drivetrain
    m_drivetrain.drive(chassisSpeeds);
    debugLogsDrive();
  } // end of execute()

  // --------------------------------------------------------------------------------------
  //
  // Debug Logging
  //
  // --------------------------------------------------------------------------------------
  public void debugLogsDrive() {
  }

  // --------------------------------------------------------------------------------------
  //
  // End
  //
  // --------------------------------------------------------------------------------------
  @Override
  public void end(boolean interrupted) {}

  // --------------------------------------------------------------------------------------
  //
  // Is Finished
  //
  // --------------------------------------------------------------------------------------
  @Override
  public boolean isFinished() {
    return false;
  }
}
