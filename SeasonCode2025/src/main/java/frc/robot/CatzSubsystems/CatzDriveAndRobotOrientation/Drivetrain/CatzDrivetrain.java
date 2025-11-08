package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.OdometryObservation;
import frc.robot.Commands.DriveAndRobotOrientationCmds.HolonomicDriveController;
import frc.robot.Robot;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.EqualsUtil;

import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// Drive train subsystem for swerve drive implementation
public class CatzDrivetrain extends SubsystemBase {
  public static final CatzDrivetrain Instance = new CatzDrivetrain();

  private double distanceError = 999999.9; //meters

  private Pose2d choreoGoal = new Pose2d();
  private double choreoDistanceError = 9999999.9; //meters //set this to a high number initially just in case the trajectory thinks it's at goal as soon as it starts

  // Gyro input/output interface
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Alerts
  private final Alert gyroDisconnected;

  // Array of swerve modules representing each wheel in the drive train
  private CatzSwerveModule[] m_swerveModules = new CatzSwerveModule[4];
  private SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];

  // Swerve modules representing each corner of the robot
  public final CatzSwerveModule RT_FRNT_MODULE;
  public final CatzSwerveModule RT_BACK_MODULE;
  public final CatzSwerveModule LT_BACK_MODULE;
  public final CatzSwerveModule LT_FRNT_MODULE;

  private HolonomicDriveController hoController = DriveConstants.getNewHolController();

  // TODO Remmove this if we are not going to use it
  // private SwerveSetpoint currentSetpoint =
  //     new SwerveSetpoint(
  //         new ChassisSpeeds(),
  //         new SwerveModuleState[] {
  //           new SwerveModuleState(),
  //           new SwerveModuleState(),
  //           new SwerveModuleState(),
  //           new SwerveModuleState()
  //         });
  // private final SwerveSetpointGenerator swerveSetpointGenerator;

  private final Field2d field;

  private Pose2d pidGoalPose = new Pose2d();

  private CatzDrivetrain() {

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
    RT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_FR], MODULE_NAMES[INDEX_FR]);
    RT_BACK_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_BR], MODULE_NAMES[INDEX_BR]);
    LT_BACK_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_BL], MODULE_NAMES[INDEX_BL]);
    LT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_FL], MODULE_NAMES[INDEX_FL]);

    // Assign swerve modules to the array for easier access
    m_swerveModules[INDEX_FR] = RT_FRNT_MODULE;
    m_swerveModules[INDEX_BR] = RT_BACK_MODULE;
    m_swerveModules[INDEX_BL] = LT_BACK_MODULE;
    m_swerveModules[INDEX_FL] = LT_FRNT_MODULE;

    // swerveSetpointGenerator =
    //     new SwerveSetpointGenerator(DriveConstants.SWERVE_KINEMATICS, DriveConstants.MODULE_TRANSLATIONS);
    // TODO Remmove this if we are not going to use it

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          Logger.recordOutput("Drive/targetPos", pose);
          field.getObject("target pose").setPose(pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
        });
  }

  Pose2d pose = new Pose2d();

  public double getDistanceError(){
    return distanceError;
  }

  public void setDistanceError(double d){
    this.distanceError = d;
  }

  @Override
  public void periodic() {
    // ----------------------------------------------------------------------------------------------------
    // Update inputs (sensors/encoders) for code logic and advantage kit
    // ----------------------------------------------------------------------------------------------------
    for (CatzSwerveModule module : m_swerveModules) {
      module.periodic();
    }

    Logger.recordOutput("Drive/DistanceError", distanceError);

    pose = pose.interpolate(CatzRobotTracker.Instance.getEstimatedPose(), 0.05);
    Logger.recordOutput("CatzRobotTracker/interlated pose", pose);

    // -----------------------------------------------------------------------------------------------------
    // Attempt to update gyro inputs and log
    // -----------------------------------------------------------------------------------------------------
    try {
      gyroIO.updateInputs(gyroInputs);
    } catch (Exception e) {

    }
    Logger.processInputs("RealInputs/Drive/gyro ", gyroInputs);
    // NOTE Gyro needs to be firmly mounted to rio for accurate results.
    // Set Gyro Disconnect alert to go off when gyro is disconnected
    if (Robot.isReal()) {
      gyroDisconnected.set(!gyroInputs.gyroConnected);
    }

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
    CatzRobotTracker.Instance.addOdometryObservation(observation);

    // Update current velocities use gyro when possible
    Twist2d robotRelativeVelocity = getTwist2dSpeeds();
    robotRelativeVelocity.dtheta =
        gyroInputs.gyroConnected
            ? Math.toRadians(gyroInputs.gyroYawVel)
            : robotRelativeVelocity.dtheta;
    CatzRobotTracker.Instance.addVelocityData(robotRelativeVelocity);

    // --------------------------------------------------------------
    // Logging
    // --------------------------------------------------------------
    SmartDashboard.putNumber("Heading", getGyroHeading());
    Logger.recordOutput("Drive/Odometry module states", getModuleStates());
    Logger.recordOutput("Drive/Odometry wheel positions", wheelPositions);
    Logger.recordOutput("Drive/Odometry robot velocity", robotRelativeVelocity);

  } // end of drivetrain periodic

  // --------------------------------------------------------------------------------------------------------------------------
  //
  //          Driving methods
  //
  // --------------------------------------------------------------------------------------------------------------------------
  public void drive(ChassisSpeeds chassisSpeeds) {
    // System.out.println("speeed; " + chassisSpeeds);
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

    Logger.recordOutput("Drive/chassispeeds", descreteSpeeds);
    Logger.recordOutput("Drive/modulestates", optimizedDesiredStates);
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
    for (CatzSwerveModule module : m_swerveModules) {
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
  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(m_swerveModules).mapToDouble(CatzSwerveModule::getPositionRads).toArray();
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : m_swerveModules) {
      driveVelocityAverage += module.getCharacterizationVelocityRadPerSec();
    }
    return driveVelocityAverage / 4.0;
  }

  /**
   * Returns command that orients all modules to {@code orientation}, ending when the modules have
   * rotated.
   */
  public Command orientModules(Rotation2d orientation) {
    return orientModules(new Rotation2d[] {orientation, orientation, orientation, orientation});
  }

  /**
   * Returns command that orients all modules to {@code orientations[]}, ending when the modules
   * have rotated.
   */
  public Command orientModules(Rotation2d[] orientations) {
    return run(() -> {
          for (int i = 0; i < orientations.length; i++) {
            m_swerveModules[i].setModuleAngleAndVelocity(
                new SwerveModuleState(0.0, orientations[i]));
            // new SwerveModuleState(0.0, new Rotation2d()));
          }
        })
        .until(
            () ->
                Arrays.stream(m_swerveModules)
                    .allMatch(
                        module ->
                            EqualsUtil.epsilonEquals(
                                module.getAngle().getDegrees(),
                                module.getModuleState().angle.getDegrees(),
                                2.0)))
        .withName("Orient Modules");
  }

  /** Set Neutral mode for all swerve modules */
  public void setDriveNeutralMode(NeutralModeValue type) {
    for (CatzSwerveModule module : m_swerveModules) {
      module.setNeutralModeDrive(type);
    }
  }

  /** Set coast mode for all swerve modules */
  public void setSteerNeutralMode(NeutralModeValue type) {
    for (CatzSwerveModule module : m_swerveModules) {
      module.setNeutralModeSteer(type);
    }
  }

  public boolean closeEnoughToRaiseElevator(){
    return choreoDistanceError <= DriveConstants.PREDICT_DISTANCE_SCORE;
  }

  public boolean closeEnoughToStartIntake(){
    return choreoDistanceError <= DriveConstants.PREDICT_DISTANCE_INTAKE;
  }

  /** command to cancel running auto trajectories */
  public Command cancelTrajectory() {
    Command cancel = new InstantCommand();
    cancel.addRequirements(this);
    return cancel;
  }

  public void resetDriveEncs() {
    for (CatzSwerveModule module : m_swerveModules) {
      module.resetDriveEncs();
    }
  }

  /**
   * Make sure to run this before every new trajectory
   */
  public void followChoreoTrajectoryInit(AutoTrajectory traj){
    hoController = DriveConstants.getNewHolController();
    choreoGoal = traj.getFinalPose().get();
  }

  /**
   * This function only runs the "execute" portion of a command. Initialization and ending should be done elsewhere.
   *
   * @param sample
   */
  public void followChoreoTrajectoryExecute(SwerveSample sample){
    Trajectory.State state = new Trajectory.State(
      sample.t,
      Math.hypot(sample.vx,sample.vy),
      0.0,
      new Pose2d(new Translation2d(sample.x, sample.y), Rotation2d.fromRadians(Math.atan2(sample.vy, sample.vx))),
      0.0
    );

    Pose2d curPose = CatzRobotTracker.Instance.getEstimatedPose();
    ChassisSpeeds adjustedSpeeds = hoController.calculate(curPose, state, Rotation2d.fromRadians(sample.heading));

    choreoDistanceError = curPose.minus(choreoGoal).getTranslation().getNorm();

    Logger.recordOutput("Target Auton Pose", new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading)));
    drive(adjustedSpeeds);
  }

  public boolean isRobotAtPoseChoreo(){
    return choreoDistanceError <= AutonConstants.ACCEPTABLE_DIST_METERS;
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
    return gyroInputs.gyroAngle; // Negative on Forte due to instalation, gyro's left is not robot left
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
  @AutoLogOutput(key = "Drive/MeasuredSpeeds")
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

  public Pose2d getPIDGoalPose(){
    return pidGoalPose;
  }

  public void setPIDGoalPose(Pose2d p){
    this.pidGoalPose = p;
  }
}
