// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Utilities.GeomUtil;
import java.util.NoSuchElementException;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;

@ExtensionMethod({GeomUtil.class})
public class CatzRobotTracker {
  private static final double POSE_BUFFER_SIZE_SEC = 2.0;
  private static final Matrix<N3, N1> ODOMETRY_STD_DEVS =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

  private static CatzRobotTracker instance;

  public static CatzRobotTracker getInstance() {
    if (instance == null) instance = new CatzRobotTracker();
    return instance;
  }

  // ------------------------------------------------------------------------------------------------------
  //  Pose estimation Members
  // ------------------------------------------------------------------------------------------------------
  @Getter
  @AutoLogOutput(key = "CatzRobotTracker/PureOdometryPose")
  private Pose2d odometryPose = new Pose2d();

  @Getter
  @AutoLogOutput(key = "CatzRobotTracker/EstimatedPose")
  private Pose2d estimatedPose = new Pose2d();

  private final TimeInterpolatableBuffer<Pose2d> POSE_BUFFER =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SEC);
  private final Matrix<N3, N1> TRACKER_STD_DEVS = new Matrix<>(Nat.N3(), Nat.N1());

  // Odometry
  private final SwerveDriveKinematics KINEMATICS;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  @Getter
  private SwerveModuleState[] currentModuleStates =
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

  // ------------------------------------------------------------------------------------------------------
  //
  //  Constructor
  //
  // ------------------------------------------------------------------------------------------------------
  private CatzRobotTracker() {
    for (int i = 0; i < 3; ++i) {
      TRACKER_STD_DEVS.set(i, 0, Math.pow(ODOMETRY_STD_DEVS.get(i, 0), 2));
    }
    KINEMATICS = DriveConstants.SWERVE_KINEMATICS;
  }

  // ------------------------------------------------------------------------------------------------------
  //
  //  Pose Estimation adder methods
  //
  // ------------------------------------------------------------------------------------------------------
  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {
    currentModuleStates = observation.moduleStates;
    // Calculate twist from last wheel positions to current wheel positions, useful for when gyro is
    // disabled
    Twist2d twist = KINEMATICS.toTwist2d(lastWheelPositions, observation.wheelPositions());
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
    POSE_BUFFER.addSample(observation.timestamp(), odometryPose);
    // Calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(twist);

    // Collect Velocity and Accerleration values
    var chassisSpeeds = KINEMATICS.toChassisSpeeds(observation.moduleStates);
    robotAccelerations =
        new Twist2d(
            (chassisSpeeds.vxMetersPerSecond - m_lastChassisSpeeds.vxMetersPerSecond)
                / observation.timestamp,
            (chassisSpeeds.vyMetersPerSecond - m_lastChassisSpeeds.vyMetersPerSecond)
                / observation.timestamp,
            (chassisSpeeds.omegaRadiansPerSecond - m_lastChassisSpeeds.omegaRadiansPerSecond)
                / observation.timestamp);
    m_lastChassisSpeeds = chassisSpeeds;
  } // end of addOdometryObservation

  /** Add Vision Observation */
  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (POSE_BUFFER.getInternalBuffer().lastKey() - POSE_BUFFER_SIZE_SEC
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    System.out.println("exceptio");
    // Get odometry based pose at timestamp
    var sample = POSE_BUFFER.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }
    // System.out.println("empty sample");

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
      double stdDev = TRACKER_STD_DEVS.get(row, 0);
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
  } // end of addVisionObservation(OdometryObservation observation)

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

  public void addFeedFowardData() {}

  public void addTrajectoryVelocityData(Twist2d robotVelocity) {
    trajectoryVelocity = robotVelocity;
  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(Pose2d initialPose) {
    System.out.println(initialPose.getRotation().getDegrees());
    estimatedPose = initialPose;
    odometryPose = initialPose;
    POSE_BUFFER.clear();
  }

  // ------------------------------------------------------------------------------------------------------
  //
  //  CatzRobotTracker Getters
  //
  // ------------------------------------------------------------------------------------------------------
  @AutoLogOutput(key = "CatzRobotTracker/FieldVelocity")
  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  /**
   * Predicts what our pose will be in the future. Allows separate translation and rotation
   * lookaheads to account for varying latencies in the different measurements.
   *
   * @param translationLookaheadS The lookahead time for the translation of the robot
   * @param rotationLookaheadS The lookahead time for the rotation of the robot
   * @return The predicted pose.
   */
  @AutoLogOutput(key = "CatzRobotTracker/PredictedPose")
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    Twist2d velocity = DriverStation.isAutonomousEnabled() ? trajectoryVelocity : robotVelocity;
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                velocity.dx * translationLookaheadS,
                velocity.dy * translationLookaheadS,
                Rotation2d.fromRadians(velocity.dtheta * rotationLookaheadS)));
  }

  @AutoLogOutput(key = "CatzRobotTracker/RecordedChassisSpeeds")
  public ChassisSpeeds getRobotChassisSpeeds() {
    return m_lastChassisSpeeds;
  }

  /********************************************************************************************************************************
   *
   * Odometry and Vision Record types
   *
   ********************************************************************************************************************************/
  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions,
      SwerveModuleState[] moduleStates,
      Rotation2d gyroAngle,
      double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
