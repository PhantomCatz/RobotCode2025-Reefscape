// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.RobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.RobotTracker.OdometryObservation;
import frc.robot.Robot;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.EqualsUtil;

import java.util.Arrays;

// Drive train subsystem for swerve drive implementation
public class driverain extends SubsystemBase {

  // Gyro input/output interface
  private final GyroIO gyroIO;

  // Position, odmetetry, and velocity estimator
  private final RobotTracker tracker = RobotTracker.getInstance();

  // Alerts
  private final Alert gyroDisconnected;

  // Array of swerve modules representing each wheel in the drive train
  private SwerveModule[] m_swerveModules = new SwerveModule[4];
  private SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];

  // Swerve modules representing each corner of the robot
  public final SwerveModule RT_FRNT_MODULE;
  public final SwerveModule RT_BACK_MODULE;
  public final SwerveModule LT_BACK_MODULE;
  public final SwerveModule LT_FRNT_MODULE;


  private final Field2d field;

  public driverain() {

    // Gyro Instantiation
    switch (CatzConstants.hardwareMode) {
      case REAL:
        gyroIO = new GyroIOPigeon();
        break;
      case REPLAY:
        gyroIO = new GyroIOPigeon() {};
        break;
      default:
        gyroIO = null;
        break;
    }

    gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.kWarning);

    // Create swerve modules for each corner of the robot
    RT_FRNT_MODULE = new SwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_FR], MODULE_NAMES[INDEX_FR]);
    RT_BACK_MODULE = new SwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_BR], MODULE_NAMES[INDEX_BR]);
    LT_BACK_MODULE = new SwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_BL], MODULE_NAMES[INDEX_BL]);
    LT_FRNT_MODULE = new SwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_FL], MODULE_NAMES[INDEX_FL]);

    // Assign swerve modules to the array for easier access
    m_swerveModules[INDEX_FR] = RT_FRNT_MODULE;
    m_swerveModules[INDEX_BR] = RT_BACK_MODULE;
    m_swerveModules[INDEX_BL] = LT_BACK_MODULE;
    m_swerveModules[INDEX_FL] = LT_FRNT_MODULE;


    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
        });



    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
        });
  }

  Pose2d pose = new Pose2d();

  @Override
  public void periodic() {
    // ----------------------------------------------------------------------------------------------------
    // Update inputs (sensors/encoders) for code logic and advantage kit
    // ----------------------------------------------------------------------------------------------------
    for (SwerveModule module : m_swerveModules) {
      module.periodic();
    }

    pose = pose.interpolate(tracker.getEstimatedPose(), 0.05);

    // -----------------------------------------------------------------------------------------------------
    // Attempt to update gyro inputs and log
    // -----------------------------------------------------------------------------------------------------
    // NOTE Gyro needs to be firmly mounted to rio for accurate results.
    // Set Gyro Disconnect alert to go off when gyro is disconnected


    // ----------------------------------------------------------------------------------------------------
    // Swerve drive Odometry and Velocity updates
    // ----------------------------------------------------------------------------------------------------
    SwerveModulePosition[] wheelPositions = getModulePositions();
    // Grab latest gyro measurments
    Rotation2d gyroAngle2d =
        (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.SIM)
            ? null
            : getRotation2d();

    // Add observations to robot tracker
    OdometryObservation observation = new OdometryObservation(
                                                    wheelPositions,
                                                    getModuleStates(),
                                                    gyroAngle2d,
                                                    Timer.getFPGATimestamp()
                                          );
    RobotTracker.getInstance().addOdometryObservation(observation);

    // Update current velocities use gyro when possible
    Twist2d robotRelativeVelocity = getTwist2dSpeeds();

    // --------------------------------------------------------------
    // Logging
    // --------------------------------------------------------------
    SmartDashboard.putNumber("Heading", getGyroHeading());
  } // end of drivetrain periodic

  // --------------------------------------------------------------------------------------------------------------------------
  //
  //          Driving methods
  //
  // --------------------------------------------------------------------------------------------------------------------------
  public void drive(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds descreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, CatzConstants.LOOP_TIME);
    // --------------------------------------------------------
    // Convert chassis speeds to individual module states and set module states
    // --------------------------------------------------------
    SwerveModuleState[] unoptimizedModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(descreteSpeeds);
    // --------------------------------------------------------
    // Scale down wheel speeds
    // --------------------------------------------------------
    SwerveDriveKinematics.desaturateWheelSpeeds(unoptimizedModuleStates, DriveConstants.DRIVE_CONFIG.maxLinearVelocity());
    // --------------------------------------------------------
    // Optimize Wheel Angles
    // --------------------------------------------------------
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state that prevents it from overturn, useful for logging
      optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(unoptimizedModuleStates[i]);

      // Set module states to each of the swerve modules
      m_swerveModules[i].setModuleAngleAndVelocity(optimizedDesiredStates[i]);
    }

    // --------------------------------------------------------
    // Logging
    // --------------------------------------------------------

  }
  public void d(ChassisSpeeds s){
    //nothing
  }

  public void simpleDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates =
        DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state that prevents it from overturn, useful for logging
      optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(moduleStates[i]);

      // Set module states to each of the swerve modules
      m_swerveModules[i].setModuleAngleAndVelocity(optimizedDesiredStates[i]);
    }
  }

  /** Create a command to stop driving */
  public void stopDriving() {
    for (SwerveModule module : m_swerveModules) {
      module.stopDriving();
      module.setSteerPower(0.0);
    }
  }

  /** Runs in a circle at omega. */
  public void runWheelRadiusCharacterization(double omegaSpeed) {
    simpleDrive(new ChassisSpeeds(0.0, 0.0, omegaSpeed));
  }

  /** Disables the characterization mode. */
  public void endCharacterization() {
    stopDriving();
  }

  /** Runs forwards at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    simpleDrive(new ChassisSpeeds(0.0, 0.0, input));
  }

  // -----------------------------------------------------------------------------------------------------------
  //
  //      Drivetrain Misc Methods
  //
  // -----------------------------------------------------------------------------------------------------------

  /** Set Neutral mode for all swerve modules */
  public void setDriveNeutralMode(NeutralModeValue type) {
    for (SwerveModule module : m_swerveModules) {
      module.setNeutralModeDrive(type);
    }
  }

  /** Set coast mode for all swerve modules */
  public void setSteerNeutralMode(NeutralModeValue type) {
    for (SwerveModule module : m_swerveModules) {
      module.setNeutralModeSteer(type);
    }
  }

  /** command to cancel running auto trajectories */
  public Command cancelTrajectory() {
    Command cancel = new InstantCommand();
    cancel.addRequirements(this);
    return cancel;
  }

  public void resetDriveEncs() {
    for (SwerveModule module : m_swerveModules) {
      module.resetDriveEncs();
    }
  }

  // -----------------------------------------------------------------------------------------------------------
  //
  //      Drivetrain Getters
  //
  // -----------------------------------------------------------------------------------------------------------
  /**
   * Dependant on the installation of the gyro, the value of this method may be negative
   *
   * @return The Heading of the robot dependant on where it's been instantiated
   */
  private double getGyroHeading() {
    return gyroIO.getAngle(); // Negative on Forte due to instalation, gyro's left is not robot left
  }

  /** Get the Rotation2d object based on the gyro angle */
  private Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getGyroHeading());
  }

  /** Get an array of swerve module states */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < m_swerveModules.length; i++) {
      moduleStates[i] = m_swerveModules[i].getModuleState();
    }
    return moduleStates;
  }

  /** Returns the measured speeds of the robot in the robot's frame of reference. */
  private Twist2d getTwist2dSpeeds() {
    return DriveConstants.SWERVE_KINEMATICS.toTwist2d(getModulePositions());
  }

  /** Get an array of swerve module positions */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < m_swerveModules.length; i++) {
      modulePositions[i] = m_swerveModules[i].getModulePosition();
    }
    return modulePositions;
  }

  /** Map Circle orientation for wheel radius characterization */
  public static Rotation2d[] getCircleOrientations() {
    return Arrays.stream(DriveConstants.MODULE_TRANSLATIONS)
        .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
        .toArray(Rotation2d[]::new);
  }
}
