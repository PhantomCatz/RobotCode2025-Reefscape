
package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.INDEX_BL;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.INDEX_BR;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.INDEX_FL;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.INDEX_FR;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.MODULE_NAMES;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker.OdometryObservation;
import frc.robot.Robot;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.EqualsUtil;
import frc.robot.Utilities.LocalADStarAK;
import frc.robot.Utilities.MotorUtil.NeutralMode;

// Drive train subsystem for swerve drive implementation
public class CatzDrivetrain extends SubsystemBase {
    
    // Gyro input/output interface
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    // Position, odmetetry, and velocity estimator
    private final CatzRobotTracker tracker = CatzRobotTracker.getInstance();

    // Alerts
    private final Alert gyroDisconnected;

    // Array of swerve modules representing each wheel in the drive train
    private CatzSwerveModule[] m_swerveModules = new CatzSwerveModule[4];
    private SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];

    // Swerve modules representing each corner of the robot
    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;

    //Swerve setpoint generator
    private SwerveSetpointGenerator swerveSetpointGenerator = new SwerveSetpointGenerator(DriveConstants.TRAJECTORY_CONFIG, DriveConstants.DRIVE_CONFIG.maxAngularVelocity());
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(tracker.getRobotChassisSpeeds(), tracker.getCurrentModuleStates(), DriveFeedforwards.zeros(4));


    private final Field2d field;


    public CatzDrivetrain() {

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
        LT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_FL], MODULE_NAMES[INDEX_FL]);
        LT_BACK_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_BL], MODULE_NAMES[INDEX_BL]);
        RT_BACK_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_BR], MODULE_NAMES[INDEX_BR]);
        RT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.MODULE_CONFIGS[INDEX_FR], MODULE_NAMES[INDEX_FR]);

        // Assign swerve modules to the array for easier access
        m_swerveModules[INDEX_FL] = LT_FRNT_MODULE;
        m_swerveModules[INDEX_BL] = LT_BACK_MODULE;
        m_swerveModules[INDEX_BR] = RT_BACK_MODULE;
        m_swerveModules[INDEX_FR] = RT_FRNT_MODULE;


        //---------------------------------------------------------------------------------
        // Pathplanner Logging
        //---------------------------------------------------------------------------------
        // Configure logging trajectories to advantage kit
        Pathfinding.setPathfinder(new LocalADStarAK());
        
        // PathPlanner Debug
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Logger.recordOutput("Drive/targetPos", pose);
            CatzRobotTracker.getInstance().addTrajectorySetpointData(pose);
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
    }



    @Override
    public void periodic() {
        //----------------------------------------------------------------------------------------------------
        // Update inputs (sensors/encoders) for code logic and advantage kit
        //----------------------------------------------------------------------------------------------------
        for (CatzSwerveModule module : m_swerveModules) {
            module.periodic();
        }

        //-----------------------------------------------------------------------------------------------------
        // Attempt to update gyro inputs and log
        //-----------------------------------------------------------------------------------------------------
        try {
            gyroIO.updateInputs(gyroInputs);
        } catch (Exception e) {

        }
        Logger.processInputs("inputs/Drive/gyro ", gyroInputs);    
        // NOTE Gyro needs to be firmly mounted to rio for accurate results.
        // Set Gyro Disconnect alert to go off when gyro is disconnected
        if (Robot.isReal()) {
            gyroDisconnected.set(!gyroInputs.gyroConnected);
        }

        //----------------------------------------------------------------------------------------------------
        // Swerve drive Odometry and Velocity updates
        //----------------------------------------------------------------------------------------------------
        SwerveModulePosition[] wheelPositions = getModulePositions();
        // Grab latest gyro measurments
        Rotation2d gyroAngle2d = 
                        (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.SIM)
                            ? null
                            : getRotation2d();
        
        
        // Add observations to robot tracker
        OdometryObservation observation = new OdometryObservation(wheelPositions, 
                                                                  getModuleStates(), 
                                                                  gyroAngle2d, 
                                                                  Timer.getFPGATimestamp()
        );
        CatzRobotTracker.getInstance().addOdometryObservation(observation);

        // Update current velocities use gyro when possible
        Twist2d robotRelativeVelocity = getTwist2dSpeeds();
        robotRelativeVelocity.dtheta =
            gyroInputs.gyroConnected
                ? Math.toRadians(gyroInputs.gyroYawVel)
                : robotRelativeVelocity.dtheta;
        CatzRobotTracker.getInstance().addVelocityData(robotRelativeVelocity);

        //--------------------------------------------------------------
        // Logging
        //--------------------------------------------------------------
        SmartDashboard.putNumber("Heading", getGyroHeading());
        Logger.recordOutput("Drive/Odometry module states", getModuleStates());
        Logger.recordOutput("Drive/Odometry wheel positions", wheelPositions);
        Logger.recordOutput("Drive/Odometry robot velocity", robotRelativeVelocity);


    }   //end of drivetrain periodic

    //--------------------------------------------------------------------------------------------------------------------------
    //
    //          Driving methods
    //
    //--------------------------------------------------------------------------------------------------------------------------
    public void drive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds,0.02);
        //--------------------------------------------------------
        // Convert chassis speeds to individual module states and set module states
        //--------------------------------------------------------
        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        //--------------------------------------------------------
        // Scale down wheel speeds
        //--------------------------------------------------------
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.DRIVE_CONFIG.maxLinearVelocity());
        
        //--------------------------------------------------------
        // Optimize Wheel Angles
        //--------------------------------------------------------
        for (int i = 0; i < 4; i++) {  
            // The module returns the optimized state that prevents it from overturn, useful for logging
            optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(moduleStates[i]);

            // Set module states to each of the swerve modules
            m_swerveModules[i].setModuleAngleAndVelocity(optimizedDesiredStates[i]);
        }

        //--------------------------------------------------------
        // Logging
        //--------------------------------------------------------

        Logger.recordOutput("Drive/chassispeeds", chassisSpeeds);
        Logger.recordOutput("Drive/modulestates", optimizedDesiredStates);
    }

    public void simpleDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        for (int i = 0; i < 4; i++) {  
            // The module returns the optimized state that prevents it from overturn, useful for logging
            optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(moduleStates[i]);

            // Set module states to each of the swerve modules
            m_swerveModules[i].setModuleAngleAndVelocity(optimizedDesiredStates[i]);
        }
    }

    /**  Create a command to stop driving */
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

    //-----------------------------------------------------------------------------------------------------------
    //
    //      Drivetrain Misc Methods
    //
    //-----------------------------------------------------------------------------------------------------------
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
                    //new SwerveModuleState(0.0, new Rotation2d()));
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
    
    /**  Set Neutral mode for all swerve modules */
    public void setDriveNeutralMode(NeutralModeValue type) {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setNeutralModeDrive(type);
        }
    }

    /**  Set coast mode for all swerve modules */
    public void setSteerNeutralMode(NeutralModeValue type) {
        for (CatzSwerveModule module : m_swerveModules) {
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
        for (CatzSwerveModule module : m_swerveModules) {
            module.resetDriveEncs();
        }
    }

    //-----------------------------------------------------------------------------------------------------------
    //
    //      Drivetrain Getters
    //
    //-----------------------------------------------------------------------------------------------------------
    /**
     * Dependant on the installation of the gyro, the value of this method may be negative
     * 
     * @return The Heading of the robot dependant on where it's been instantiated
     */
    private double getGyroHeading() {
        return -gyroInputs.gyroAngle; // Negative on Forte due to instalation, gyro's left is not robot left
    }

    /** Get the Rotation2d object based on the gyro angle */
    private Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getGyroHeading());
    }

    /**  Get an array of swerve module states */
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

    /**  Get an array of swerve module positions */
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
