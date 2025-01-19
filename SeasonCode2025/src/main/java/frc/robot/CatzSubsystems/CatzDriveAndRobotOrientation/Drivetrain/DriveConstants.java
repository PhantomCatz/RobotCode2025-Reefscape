// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.Builder;

public class DriveConstants {
  // ---------------------------------------------------------------------------------------------------------------
  // Disabled flag for testing
  // ---------------------------------------------------------------------------------------------------------------
  public static final boolean IS_DRIVE_DISABLED = false;

  // ---------------------------------------------------------------------------------------------------------------
  // Module organizations
  // ---------------------------------------------------------------------------------------------------------------
  public static final String[] MODULE_NAMES = new String[] {"FR", "BR", "BL", "FL"};
  public static final int INDEX_FR = 0;
  public static final int INDEX_BR = 1;
  public static final int INDEX_BL = 2;
  public static final int INDEX_FL = 3;

  public static final int GYRO_ID = 0;

  // ---------------------------------------------------------------------------------------------------------------
  // Drive Subsytem Config info
  // ---------------------------------------------------------------------------------------------------------------
  public static final double DRIVE_CURRENT_LIMIT = 80.0;
  public static final double STEER_CURRENT_LIMIT = 40.0;

  // Velocity and Accerlation Constants
  private static final double kA_ANGULAR_ACCEL = 0.0;
  private static final double kA_LINEAR_ACCEL = 0.0;

  public static final DriveConfig DRIVE_CONFIG =
      switch (CatzConstants.getRobotType()) {
        case SN_TEST, SN2 ->
            DriveConfig.builder()
                .wheelRadius(Units.inchesToMeters(1.8))
                .robotLengthX(Units.inchesToMeters(29.0))
                .robotWidthY(Units.inchesToMeters(29.0))
                .bumperWidthX(Units.inchesToMeters(37))
                .bumperWidthY(Units.inchesToMeters(33))
                .maxLinearVelocity(Units.feetToMeters(17))
                .maxLinearAcceleration(Math.pow(Units.feetToMeters(17), 2)) // TODO why square
                .maxAngularVelocity(Units.degreesToRadians(600)) // Radians
                .maxAngularAcceleration(
                    Units.degreesToRadians(600)) // Radians // TODO verify angle constraints
                .build();
        case SN1, SN1_2024 ->
            new DriveConfig(
                Units.inchesToMeters(2.01834634),
                Units.inchesToMeters(29.0),
                Units.inchesToMeters(29.0),
                Units.inchesToMeters(37),
                Units.inchesToMeters(33),
                Units.feetToMeters(12.16),
                Units.feetToMeters(21.32),
                7.93,
                29.89);
      };

  public static final ModuleGainsAndRatios MODULE_GAINS_AND_RATIOS =
      switch (CatzConstants.getRobotType()) {
        case SN1 ->
            new ModuleGainsAndRatios(
                5.0,
                0.0,
                1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
                0.2,
                0.0,
                0.30,
                0.005,
                Mk4iReductions.L2_PLUS.reduction,
                Mk4iReductions.steer.reduction);
        case SN2 ->
            new ModuleGainsAndRatios(
                5.5,
                0.6,
                0.0,
                1.2, // 1.2, //TODO fix to account for non foc
                0.0,
                0.001,
                0.000,
                Mk4iReductions.L2_PLUS.reduction,
                Mk4iReductions.steer.reduction);
        case SN_TEST ->
            new ModuleGainsAndRatios(
                0.014,
                0.134,
                0.0,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L2_PLUS.reduction,
                Mk4iReductions.steer.reduction);
        case SN1_2024 ->
            new ModuleGainsAndRatios(
                5.0,
                0.0,
                1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
                0.2,
                0.0,
                0.3,
                0.005,
                Mk4iReductions.L2_PLUS.reduction,
                Mk4iReductions.steer.reduction);
      };

  // -------------------------------------------------------------------------------
  // Odometry Constants
  // -------------------------------------------------------------------------------
  public static final double ODOMETRY_FREQUENCY =
      switch (CatzConstants.getRobotType()) {
        case SN_TEST -> 50.0;
        case SN1, SN1_2024 -> 100.0;
        case SN2 -> 250.0;
      };

  // ---------------------------------------------------------------------------------------------------------------------
  // Logged Tunable PIDF values for swerve modules
  // ---------------------------------------------------------------------------------------------------------------------
  public static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP", MODULE_GAINS_AND_RATIOS.drivekP());
  public static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD", MODULE_GAINS_AND_RATIOS.drivekD());
  public static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS", MODULE_GAINS_AND_RATIOS.driveFFkS());
  public static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV", MODULE_GAINS_AND_RATIOS.driveFFkV());
  public static final LoggedTunableNumber steerkP =
      new LoggedTunableNumber("Drive/Module/steerkP", MODULE_GAINS_AND_RATIOS.steerkP());
  public static final LoggedTunableNumber steerkD =
      new LoggedTunableNumber("Drive/Module/steerkD", MODULE_GAINS_AND_RATIOS.steerkD());

  public static final ModuleIDsAndCurrentLimits[] MODULE_CONFIGS =
      switch (CatzConstants.getRobotType()) {
        case SN2 ->
            new ModuleIDsAndCurrentLimits[] {
              new ModuleIDsAndCurrentLimits(1, 2, 9, 1.4196464857 / Math.PI / 2 + 0.5, false),
              new ModuleIDsAndCurrentLimits(3, 4, 10, 4.6208462275 / Math.PI / 2 + 0.5, false),
              new ModuleIDsAndCurrentLimits(5, 6, 11, 0.6691969510 / Math.PI / 2, false),
              new ModuleIDsAndCurrentLimits(7, 8, 12, 2.0568857418 / Math.PI / 2, false)
            };

        case SN1 ->
            new ModuleIDsAndCurrentLimits[] {
              new ModuleIDsAndCurrentLimits(7, 8, 14, -0.7688, false), // FL
              new ModuleIDsAndCurrentLimits(5, 6, 13, -0.632, false), // BL
              new ModuleIDsAndCurrentLimits(3, 4, 12, -1.6421, false), // BR
              new ModuleIDsAndCurrentLimits(1, 2, 11, 0.8887, false) // FR
            };
        case SN_TEST ->
            new ModuleIDsAndCurrentLimits[] {
              new ModuleIDsAndCurrentLimits(1, 2, 9, 0.0, false),
              new ModuleIDsAndCurrentLimits(3, 4, 8, 0.0, false),
              new ModuleIDsAndCurrentLimits(5, 6, 7, 0.0, false),
              new ModuleIDsAndCurrentLimits(7, 8, 6, 0.0, false)
            };

        case SN1_2024 ->
            new ModuleIDsAndCurrentLimits[] {
              new ModuleIDsAndCurrentLimits(1, 2, 1, 0.7066, false), // FL
              new ModuleIDsAndCurrentLimits(3, 4, 2, 1.0682, false), // BL
              new ModuleIDsAndCurrentLimits(5, 6, 3, 1.2969, false), // BR
              new ModuleIDsAndCurrentLimits(7, 8, 4, 1.4919, false) // FR
            };
      };

  // -----------------------------------------------------------------------------------------------------------------------------
  //
  //      Drivebase controller/object definements
  //
  // -----------------------------------------------------------------------------------------------------------------------------
  public static final PathConstraints PATHFINDING_CONSTRAINTS =
      new PathConstraints( // 540 // 720
          DRIVE_CONFIG.maxLinearVelocity,
              DRIVE_CONFIG.maxLinearAcceleration, // max vel causing messup
          DRIVE_CONFIG.maxAngularVelocity, DRIVE_CONFIG.maxAngularAcceleration);

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(DRIVE_CONFIG.robotLengthX(), -DRIVE_CONFIG.robotWidthY())
            .div(2.0), // RT FRONT
        new Translation2d(-DRIVE_CONFIG.robotLengthX(), -DRIVE_CONFIG.robotWidthY())
            .div(2.0), // RT BACK
        new Translation2d(-DRIVE_CONFIG.robotLengthX(), DRIVE_CONFIG.robotWidthY())
            .div(2.0), // LT BACK
        new Translation2d(DRIVE_CONFIG.robotLengthX(), DRIVE_CONFIG.robotWidthY())
            .div(2.0) // LT FRONT
      };

  public static final Translation2d[] TRAJECTORY_MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(DRIVE_CONFIG.robotLengthX(), DRIVE_CONFIG.robotWidthY())
            .div(2.0), // LT FRONT
        new Translation2d(DRIVE_CONFIG.robotLengthX(), -DRIVE_CONFIG.robotWidthY())
            .div(2.0), // RT FRONT
        new Translation2d(-DRIVE_CONFIG.robotLengthX(), DRIVE_CONFIG.robotWidthY())
            .div(2.0), // LT BACK
        new Translation2d(-DRIVE_CONFIG.robotLengthX(), -DRIVE_CONFIG.robotWidthY())
            .div(2.0), // RT BACK
      };

  // calculates the orientation and speed of individual swerve modules when given
  // the motion of the whole robot
  public static final SwerveDriveKinematics TRAJECTORY_SWERVE_KINEMATICS =
      new SwerveDriveKinematics(TRAJECTORY_MODULE_TRANSLATIONS);
  public static final SwerveDriveKinematics SWERVE_KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  // -----------------------------------------------------------------------------------------------------------------------------
  //
  //      Trajectory Helpers
  //
  // -----------------------------------------------------------------------------------------------------------------------------
  public static HolonomicDriveController getNewHolController() {
    return new HolonomicDriveController(
        new PIDController(10.0, 0.0, 0.1),
        new PIDController(10.0, 0.0, 0.1),
        new ProfiledPIDController(
            5,
            0,
            0,
            new TrapezoidProfile.Constraints(
                DRIVE_CONFIG.maxAngularVelocity, DRIVE_CONFIG.maxAngularAcceleration)));
  }

  public static PathFollowingController getNewPathFollowingController() {
    return new PPHolonomicDriveController(
        new PIDConstants(10.0, 0.0, 0.1), new PIDConstants(4.0, 0.0, 0.1), 0.02);
  }

  public static final PathFollowingController PATH_FOLLOWING_CONTROLLER =
      getNewPathFollowingController();

  public static final ChassisSpeeds NON_ZERO_CHASSIS_SPEED = new ChassisSpeeds(1, 1, 0);

  public static final double ROBOT_MASS = 68.0;
  public static final double ROBOT_MOI =
      (1.0 / 12.0)
          * ROBOT_MASS
          * (Math.pow(DRIVE_CONFIG.bumperWidthX(), 2)
              + Math.pow(
                  DRIVE_CONFIG.bumperWidthY(),
                  2)); // ROBOT_MASS * (2/2) * (kA_ANGULAR_ACCEL/kA_LINEAR_ACCEL); // TODO need to
  // recaculate with formula on Pathplanner
  public static final double TREAD_COEF_FRICTION = 1.542;

  public static final ModuleConfig TRAJECTORY_MODULE_CONFIG =
      new ModuleConfig(
          DRIVE_CONFIG.wheelRadius(),
          DRIVE_CONFIG.maxLinearVelocity()
              * 0.8, // TODO possibly need to scale down to prevent wheel slip
          TREAD_COEF_FRICTION,
          DCMotor.getKrakenX60(1).withReduction(MODULE_GAINS_AND_RATIOS.driveReduction()),
          DRIVE_CURRENT_LIMIT,
          1);

  public static final RobotConfig TRAJECTORY_CONFIG =
      new RobotConfig(
          ROBOT_MASS, ROBOT_MOI, TRAJECTORY_MODULE_CONFIG, TRAJECTORY_MODULE_TRANSLATIONS);

  // -----------------------------------------------------------------------------------------------------------------------------
  //
  //      Simulation helpers
  //
  // -----------------------------------------------------------------------------------------------------------------------------

  /****************************************************************************************
   *
   * Record and Enum types
   *
   *******************************************************************************************/
  public record ModuleIDsAndCurrentLimits(
      int driveID,
      int steerID,
      int absoluteEncoderChannel,
      double absoluteEncoderOffset,
      boolean encoderInverted) {}

  public record ModuleGainsAndRatios(
      double driveFFkS,
      double driveFFkV,
      double driveFFkT,
      double drivekP,
      double drivekD,
      double steerkP,
      double steerkD,
      double driveReduction,
      double steerReduction) {}

  @Builder
  public record DriveConfig(
      double wheelRadius,
      double robotLengthX,
      double robotWidthY,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(robotLengthX / 2.0, robotWidthY / 2.0);
    }
  }

  public enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L2_16t(
        (50.0 / 16.0)
            * (17.0 / 27.0)
            * (45.0 / 15.0)), // SDS mk4i L2 ratio reduction plus 16 tooth pinion
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),

    L2_PLUS(6.75 * (14.0 / 16.0)),

    steer((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
