// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.CatzSubsystems.DriveAndRobotOrientation;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CatzConstants;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker.AimingParameters;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker.FlywheelSpeeds;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.GeomUtil;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.NoteVisualizer;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Constants;


@ExtensionMethod({GeomUtil.class})
public class CatzRobotTracker {

  private static final LoggedTunableNumber autoLookahead = new LoggedTunableNumber("CatzRobotTracker/AutoLookahead", 0.5);
  private static final LoggedTunableNumber lookahead = new LoggedTunableNumber("CatzRobotTracker/lookaheadS", 0.35);
  private static final LoggedTunableNumber superPoopLookahead = new LoggedTunableNumber("CatzRobotTracker/SuperPoopLookahead", 0.1);
  private static final LoggedTunableNumber closeShootingZoneFeet = new LoggedTunableNumber("CatzRobotTracker/CloseShootingZoneFeet", 10.0);

  private static final double poseBufferSizeSeconds = 2.0;


  private static CatzRobotTracker instance;
  public static CatzRobotTracker getInstance() {
    if (instance == null) instance = new CatzRobotTracker();
    return instance;
  }

  //------------------------------------------------------------------------------------------------------
  //
  //  Pose estimation Members
  //
  //------------------------------------------------------------------------------------------------------
  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();
  private Pose2d trajectorySetpointPose = new Pose2d();
  @AutoLogOutput @Getter @Setter private double trajectoryAmtCompleted = 0.0;

  
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
  @Getter @Setter private Pose2d trajectorySetpoint = new Pose2d();
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  public static final Matrix<N3, N1> odometryStateStdDevs =
      switch (CatzConstants.getRobotType()) {
        default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
      };

  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          }
  );
  @Getter private SwerveModuleState[] currentModuleStates = 
      new SwerveModuleState[] {
        new SwerveModuleState(), 
        new SwerveModuleState(), 
        new SwerveModuleState(), 
        new SwerveModuleState()
  };
  private Rotation2d lastGyroAngle = new Rotation2d();
  private Twist2d robotVelocity = new Twist2d();
  private Twist2d robotAccelerations = new Twist2d();
  private Twist2d trajectoryVelocity = new Twist2d();
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();

  @Setter private BooleanSupplier lookaheadDisable = () -> false;
  //------------------------------------------------------------------------------------------------------
  //
  //  Constructor
  //
  //------------------------------------------------------------------------------------------------------
  private CatzRobotTracker() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    kinematics = DriveConstants.swerveDriveKinematics;

    // Setup NoteVisualizer
    NoteVisualizer.setRobotPoseSupplier(this::getEstimatedPose);
  }

  //------------------------------------------------------------------------------------------------------
  //
  //  Pose Estimation adder methods
  //
  //------------------------------------------------------------------------------------------------------
  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {
    currentModuleStates = observation.moduleStates;
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    // Check gyro connected
    if (observation.gyroAngle != null) {
      // Update dtheta for twist if gyro connected
      twist =
          new Twist2d(
              twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
      lastGyroAngle = observation.gyroAngle();
    }
    // Add twist to odometry pose
    odometryPose = odometryPose.exp(twist);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // Calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(twist);

    // Collect Velocity and Accerleration values
    var chassisSpeeds = kinematics.toChassisSpeeds(observation.moduleStates);
    robotAccelerations = new Twist2d(
            (chassisSpeeds.vxMetersPerSecond - m_lastChassisSpeeds.vxMetersPerSecond) / observation.timestamp,
            (chassisSpeeds.vyMetersPerSecond - m_lastChassisSpeeds.vyMetersPerSecond) / observation.timestamp,
            (chassisSpeeds.omegaRadiansPerSecond - m_lastChassisSpeeds.omegaRadiansPerSecond)
                    / observation.timestamp);
    m_lastChassisSpeeds = chassisSpeeds;
  } //end of addOdometryObservation

  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  } //end of addVisionObservation(OdometryObservation observation)

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

  public void addTrajectoryVelocityData(Twist2d robotVelocity) {
    trajectoryVelocity = robotVelocity;
  }

  public void addTrajectorySetpointData(Pose2d targetPose) {
    this.trajectorySetpointPose = targetPose;
  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(Pose2d initialPose) {
    System.out.println(initialPose.getRotation().getDegrees());
    estimatedPose = initialPose;
    odometryPose = initialPose;
    poseBuffer.clear();
  }

  //------------------------------------------------------------------------------------------------------
  //
  //  CatzRobotTracker Geters
  //
  //------------------------------------------------------------------------------------------------------
  @AutoLogOutput(key = "CatzRobotTracker/FieldVelocity")
  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  @AutoLogOutput(key = "CatzRobotTracker/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /**
   * Predicts what our pose will be in the future. Allows separate translation and rotation
   * lookaheads to account for varying latencies in the different measurements.
   *
   * @param translationLookaheadS The lookahead time for the translation of the robot
   * @param rotationLookaheadS The lookahead time for the rotation of the robot
   * @return The predicted pose.
   */
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    Twist2d velocity = DriverStation.isAutonomousEnabled() ? trajectoryVelocity : robotVelocity;
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                velocity.dx * translationLookaheadS,
                velocity.dy * translationLookaheadS,
                Rotation2d.fromRadians(velocity.dtheta * rotationLookaheadS)));
  }

  @AutoLogOutput(key = "CatzRobotTracker/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  @AutoLogOutput(key = "CatzRobotTrakcer/RecordedChassisSpeeds")
  public ChassisSpeeds getRobotChassisSpeeds() {
    return m_lastChassisSpeeds;
  }

  //------------------------------------------------------------------------------------------------------
  //
  //  Auto Scoring Parameters
  //
  //------------------------------------------------------------------------------------------------------
  private static AimingParameters latestParameters;
  /**
   * 
   *  Credit: Jetstream 2710
   * 
   */
  public AimingParameters getAutoAimSpeakerParemeters() {

      // Collect Given Variables
      double vx = CatzRobotTracker.getInstance().getRobotChassisSpeeds().vxMetersPerSecond;
      double vy = CatzRobotTracker.getInstance().getRobotChassisSpeeds().vyMetersPerSecond;

      Pose2d robotPose = getOdometryPose();
      
      // Contstruct translation objects for vector addition
      Translation3d targetPose3d = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening);
      Translation2d targetPose = new Translation2d(targetPose3d.getX(), targetPose3d.getY()); 
      Logger.recordOutput("AutoAim/targetpose3d", targetPose3d);
      Logger.recordOutput("AutoAim/RobotPose", robotPose);
  


      // Target Robot Shooting Horizontal Angle
      Vector<N2> addedVelocity = VecBuilder.fill(vx , vy); 
      Vector<N2> robotToTarget = VecBuilder.fill(targetPose.getX() - robotPose.getX()  , targetPose.getY() - robotPose.getY());
      //Vector<N2> scaledRobotToTarget = robotToTarget.times(shooter_velocity/robotToTarget.norm());
      Vector<N2> correctVector = robotToTarget;//.minus(addedVelocity); // Account for robot velocity by subtracting vectors TODO fix later
      double feildRelTargetRad = -Math.atan(correctVector.get(1,0)/correctVector.get(0,0)); // Take arctangent to find feild relative target rotation
      
      Logger.recordOutput("AutoAim/targetAngle", Math.toDegrees(feildRelTargetRad));
      
      
      // Conversion to Turret Angle
      double targetTurretDegree = Math.toDegrees(feildRelTargetRad);    //Convert from radians to deg
      if(targetTurretDegree > 180) {         // Roll back if angle is past the softlimit
        targetTurretDegree = targetTurretDegree - 360;
      } else if(targetTurretDegree < -180) {
        targetTurretDegree = targetTurretDegree + 360;
      }
      Logger.recordOutput("AutoAim/added velocity Vector", addedVelocity.norm());
      Logger.recordOutput("AutoAim/robotToTarget Vector", robotToTarget.norm());
      Logger.recordOutput("AutoAim/Final Vector", correctVector.norm());
      Logger.recordOutput("AutoAim/After Rollback targetAngle", targetTurretDegree);



      // Target Robot Shooter Elevation Angle
      double targetDistance = targetPose.getDistance(robotPose.getTranslation());
      double elevationTicks = shooterPivotTable.get(targetDistance);

      Logger.recordOutput("AutoAim/targetDistance", targetDistance);
      Logger.recordOutput("AutoAim/TargetElevationAngle", elevationTicks);

      latestParameters =
          new AimingParameters(
              Rotation2d.fromDegrees(targetTurretDegree),
              elevationTicks,
              targetDistance,
              new FlywheelSpeeds(0, 0));
      // if (latestParameters != null) {
          // Cache previously calculated aiming parameters. Cache is invalidated whenever new
          // observations are added.
          return latestParameters;
      
      //} 
  } //end of getAutoAimSpeakerParemeters()

  public boolean inShootingZone() {
    Pose2d robot = AllianceFlipUtil.apply(getEstimatedPose());
    if (robot.getY() <= FieldConstants.Stage.ampLeg.getY()) {
      return robot.getX() <= FieldConstants.wingX;
    } else {
      return robot.getX() <= FieldConstants.FIELD_LENGTH_MTRS / 2.0 + 0.5;
    }
  }

  public boolean inCloseShootingZone() {
    return getEstimatedPose()
            .getTranslation()
            .getDistance(
                AllianceFlipUtil.apply(
                    FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
        < Units.feetToMeters(closeShootingZoneFeet.get());
  }

  //------------------------------------------------------------------------------------------------
  //  Shooter EL angle look up table key: 
  //    Param 1: Distance in meters from back wall to Center of the robot
  //    Param 2: pivot position % of max elevation units
  // TBD - how did we determine distance interval?
  // TBD - explain why two distance values
  //------------------------------------------------------------------------------------------------
  private static final InterpolatingDoubleTreeMap shooterPivotTable = new InterpolatingDoubleTreeMap();

  static {
      shooterPivotTable.put(1.478, 36.1);


      shooterPivotTable.put(2.6, 0.0);
  }

  public record AimingParameters(
      Rotation2d turretHeading,
      double shooterPivotTicks,
      double effectiveDistance,
      FlywheelSpeeds flywheelSpeeds) {}

  /**************************************************************
   * 
   * Odometry and Vision Record types
   * 
   *******************************************************************/
  public record OdometryObservation(
      SwerveDriveWheelPositions wheelPositions, SwerveModuleState[] moduleStates, Rotation2d gyroAngle, double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

  public record FlywheelSpeeds(double leftSpeed, double rightSpeed) {
    public static FlywheelSpeeds interpolate(FlywheelSpeeds t1, FlywheelSpeeds t2, double v) {
      double leftSpeed = MathUtil.interpolate(t1.leftSpeed(), t2.leftSpeed(), v);
      double rightSpeed = MathUtil.interpolate(t1.rightSpeed(), t2.rightSpeed(), v);
      return new FlywheelSpeeds(leftSpeed, rightSpeed);
    }
  }
}
