package frc.robot.Commands.DriveAndRobotOrientationCmds;

import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.Utilities.AllianceFlipUtil;

import java.util.function.Supplier;
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
    m_headingAndVelocity_X =       Math.abs(m_headingAndVelocity_X) > XboxInterfaceConstants.kDeadband ? m_headingAndVelocity_X * DriveConstants.driveConfig.maxLinearVelocity(): 0.0;
    m_headingAndVelocity_Y =       Math.abs(m_headingAndVelocity_Y) > XboxInterfaceConstants.kDeadband ? m_headingAndVelocity_Y * DriveConstants.driveConfig.maxLinearVelocity(): 0.0;
    turningVelocity =              Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband ? turningVelocity * DriveConstants.driveConfig.maxAngularVelocity(): 0.0;

    // Construct desired chassis speeds
    // Relative to field
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      m_headingAndVelocity_X, 
      m_headingAndVelocity_Y, 
      turningVelocity, 
      CatzRobotTracker.getInstance().getEstimatedPose().getRotation()
    );

    // Send new chassisspeeds object to the drivetrain
    m_drivetrain.drive(chassisSpeeds);
  } //end of execute()

  //--------------------------------------------------------------------------------------
  //
  // Debug Logging
  // 
  //--------------------------------------------------------------------------------------
  public void debugLogsDrive(){
    Logger.recordOutput("robot xspeed", xSpeed);
    Logger.recordOutput("robot yspeed", ySpeed);
    Logger.recordOutput("robot turnspeed", turningSpeed);
    Logger.recordOutput("robot orientation", m_driveTrain.getRotation2d().getRadians());
    Logger.recordOutput("chassisspeed x speed mtr sec", chassisSpeeds.vxMetersPerSecond);
    Logger.recordOutput("chassisspeed y speed mtr sec", chassisSpeeds.vyMetersPerSecond);
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

