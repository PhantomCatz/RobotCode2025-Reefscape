package frc.robot.Commands.DriveAndRobotOrientationCmds;

import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.Utilities.AllianceFlipUtil;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**************************************************************************************************
*
* TeleopDriveCmd
* 
**************************************************************************************************/

public class TeleopDriveCmd extends Command {
  // Subsystem declaration
  private final CatzDrivetrain m_drivetrain;

  // Xbox controller buttons
  private final Supplier<Double> m_headingPctOutput_X;
  private final Supplier<Double> m_headingPctOutput_Y;
  private final Supplier<Double> m_angVelocityPctOutput;

  //drive variables
  private double m_headingAndVelocity_X;
  private double m_headingAndVelocity_Y;
  private double turningVelocity;

  private ChassisSpeeds chassisSpeeds;

  //--------------------------------------------------------------------------------------
  //
  // Teleop Drive Command Constructor
  // 
  //--------------------------------------------------------------------------------------
  public TeleopDriveCmd(Supplier<Double> supplierLeftJoyX, Supplier<Double> supplierLeftJoyY, Supplier<Double> angVelocityPctOutput, CatzDrivetrain drivetrain) {
    // Chassis magnatude and direction control
    this.m_headingPctOutput_X         = supplierLeftJoyX;
    this.m_headingPctOutput_Y         = supplierLeftJoyY;
    this.m_angVelocityPctOutput       = angVelocityPctOutput;

    // subsystem assignment
    this.m_drivetrain                 = drivetrain;

    addRequirements(m_drivetrain);
  }

  //--------------------------------------------------------------------------------------
  //
  // Initialize
  // 
  //--------------------------------------------------------------------------------------
  @Override
  public void initialize() {}

  //--------------------------------------------------------------------------------------
  //
  // Execute
  // 
  //--------------------------------------------------------------------------------------
  @Override
  public void execute() {
    // Obtain realtime joystick inputs with supplier methods
    m_headingAndVelocity_X =       -m_headingPctOutput_Y.get(); 
    m_headingAndVelocity_Y =       -m_headingPctOutput_X.get(); 
    turningVelocity =              -m_angVelocityPctOutput.get(); //alliance flip shouldn't change for turing speed when switching alliances

    // Flip Directions for left joystick if alliance is red
    if(AllianceFlipUtil.shouldFlipToRed()) {
      m_headingAndVelocity_X = -m_headingAndVelocity_X;
      m_headingAndVelocity_Y = -m_headingAndVelocity_Y;
    }

    // Apply deadbands to prevent modules from receiving unintentional pwr due to joysticks having offset
    m_headingAndVelocity_X =       Math.abs(m_headingAndVelocity_X) > XboxInterfaceConstants.kDeadband ? m_headingAndVelocity_X * DriveConstants.DRIVE_CONFIG.maxLinearVelocity(): 0.0;
    m_headingAndVelocity_Y =       Math.abs(m_headingAndVelocity_Y) > XboxInterfaceConstants.kDeadband ? m_headingAndVelocity_Y * DriveConstants.DRIVE_CONFIG.maxLinearVelocity(): 0.0;
    turningVelocity =              Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband ? turningVelocity * DriveConstants.DRIVE_CONFIG.maxAngularVelocity(): 0.0;

    new Rotation2d();
    // Construct desired chassis speeds
    Rotation2d flipped = Rotation2d.fromDegrees(-CatzRobotTracker.getInstance().getEstimatedPose().getRotation().getDegrees());
    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_headingAndVelocity_X, 
                                                          m_headingAndVelocity_Y, 
                                                          turningVelocity, 
                                                          flipped);
    
    // Send new chassisspeeds object to the drivetrain
    m_drivetrain.drive(chassisSpeeds);
    debugLogsDrive();
  } //end of execute()

  //--------------------------------------------------------------------------------------
  //
  // Debug Logging
  // 
  //--------------------------------------------------------------------------------------
  public void debugLogsDrive(){
    Logger.recordOutput("Drive/robot orientation rad per sec", chassisSpeeds.omegaRadiansPerSecond);
    Logger.recordOutput("Drive/chassisspeed x speed mtr sec", chassisSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Drive/chassisspeed y speed mtr sec", chassisSpeeds.vyMetersPerSecond);
  }

  //--------------------------------------------------------------------------------------
  //
  // End
  // 
  //--------------------------------------------------------------------------------------
  @Override
  public void end(boolean interrupted) {}

  //--------------------------------------------------------------------------------------
  //
  // Is Finished
  // 
  //--------------------------------------------------------------------------------------
  @Override
  public boolean isFinished() {
    return false;
  }
}

