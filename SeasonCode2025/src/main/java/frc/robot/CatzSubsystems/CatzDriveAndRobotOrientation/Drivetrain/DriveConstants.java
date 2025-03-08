// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.HolonomicDriveController;

public class DriveConstants {
  // ---------------------------------------------------------------------------------------------------------------
  // Disabled flag for testing
  // ---------------------------------------------------------------------------------------------------------------
  public static final boolean IS_DRIVE_DISABLED = false; //bruh

  // ---------------------------------------------------------------------------------------------------------------
  // Module organizations
  // ---------------------------------------------------------------------------------------------------------------
  public static final String[] MODULE_NAMES = new String[] {"FR", "BR", "BL", "FL"};
  public static final int INDEX_FR = 0;
  public static final int INDEX_BR = 1;
  public static final int INDEX_BL = 2;
  public static final int INDEX_FL = 3;

  public static final int TRAJ_INDEX_FL = 0;
  public static final int TRAJ_INDEX_FR = 1;
  public static final int TRAJ_INDEX_BL = 2;
  public static final int TRAJ_INDEX_BR = 3;

  public static final int GYRO_ID = 10;

  // ---------------------------------------------------------------------------------------------------------------
  // Drive Subsytem Config info
  // ---------------------------------------------------------------------------------------------------------------

  public static final DriveConfig DRIVE_CONFIG =
      new DriveConfig(
        Units.inchesToMeters(1.9136),
        Units.inchesToMeters(24.2),
        Units.inchesToMeters(24.2),
        Units.inchesToMeters(32),
        Units.inchesToMeters(32),
        4,
        120,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));
        
  public static final DriveConfig SCORING_TRAJECTORY_CONFIG =
    new DriveConfig(2, 0.5, 540, 720, 0.0, 0.0, 0.0, 0.0, 0.0);


  public static final ModuleGainsAndRatios MODULE_GAINS_AND_RATIOS =
      switch (CatzConstants.getRobotType()) {
        case SN1, SN2 ->
            new ModuleGainsAndRatios(
                5.0,
                0.45,
                1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
                6.0,
                0.0,
                0.50,
                0.005,
                Mk4iReductions.L2_16t.reduction,
                Mk4iReductions.steer.reduction);
        case SN_TEST ->
            new ModuleGainsAndRatios(
                0.014,
                0.134,
                0.0,
                0.1,
                0.0,
                1.0,
                0.0,
                Mk4iReductions.L2_16t.reduction,
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

  public static final double GYRO_UPDATE_FREQUENCY =
      switch (CatzConstants.getRobotType()) {
        case SN_TEST -> 50.0;
        case SN2, SN1, SN1_2024 -> 100.0;
        //case SN2 -> 250.0;
      };

  // ---------------------------------------------------------------------------------------------------------------------
  // Logged Tunable PIDF values for swerve modules
  // ---------------------------------------------------------------------------------------------------------------------

  public static final ModuleIDs[] MODULE_CONFIGS = new ModuleIDs[4];
  static{
    switch(CatzConstants.getRobotType()){
        case SN2:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 11, 0.4599609375, false);
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 12, 0.082763671875, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 13, 0.8525390625, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 14, 0.946533203125, false);
        break;

        case SN1:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 11, 0.885986, false);
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 12, 0.3620605, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 13, 0.13012695, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 14, 0.22900, false);
        break;

        case SN_TEST:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 9, 0.0, false);
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 8, 0.0, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 7, 0.0, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 6, 0.0, false);
        break;

        case SN1_2024:
            MODULE_CONFIGS[INDEX_FR] = new ModuleIDs(1, 2, 1, 0.7066, false);
            MODULE_CONFIGS[INDEX_BR] = new ModuleIDs(3, 4, 2, 1.0682, false);
            MODULE_CONFIGS[INDEX_BL] = new ModuleIDs(5, 6, 3, 1.2969, false);
            MODULE_CONFIGS[INDEX_FL] = new ModuleIDs(7, 8, 4, 1.4919, false);
        break;
    }
  }

  // -----------------------------------------------------------------------------------------------------------------------------
  //
  //      Drivebase controller/object definements
  //
  // -----------------------------------------------------------------------------------------------------------------------------
  public static final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints( // 540 // 720
    SCORING_TRAJECTORY_CONFIG.maxLinearVelocity,
    SCORING_TRAJECTORY_CONFIG.maxLinearAcceleration,
    SCORING_TRAJECTORY_CONFIG.maxAngularVelocity,
    SCORING_TRAJECTORY_CONFIG.maxAngularAcceleration
  );

  public static final PathConstraints LEFT_RIGHT_CONSTRAINTS = new PathConstraints( // 540 // 720
    2,
    2,
    SCORING_TRAJECTORY_CONFIG.maxAngularVelocity,
    SCORING_TRAJECTORY_CONFIG.maxAngularAcceleration
  );

  public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[4];
  static {
    MODULE_TRANSLATIONS[INDEX_FR] = new Translation2d( DRIVE_CONFIG.robotLengthX(), -DRIVE_CONFIG.robotWidthY()).div(2.0);
    MODULE_TRANSLATIONS[INDEX_BR] = new Translation2d(-DRIVE_CONFIG.robotLengthX(), -DRIVE_CONFIG.robotWidthY()).div(2.0);
    MODULE_TRANSLATIONS[INDEX_BL] = new Translation2d(-DRIVE_CONFIG.robotLengthX(),  DRIVE_CONFIG.robotWidthY()).div(2.0);
    MODULE_TRANSLATIONS[INDEX_FL] = new Translation2d( DRIVE_CONFIG.robotLengthX(),  DRIVE_CONFIG.robotWidthY()).div(2.0);
  }

  // calculates the orientation and speed of individual swerve modules when given
  // the motion of the whole robot
  public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  // -----------------------------------------------------------------------------------------------------------------------------
  //
  //      Trajectory Helpers
  //
  // -----------------------------------------------------------------------------------------------------------------------------
  public static HolonomicDriveController getNewHolController() {
    return new HolonomicDriveController(
      new PIDController(8.0, 0.0, 0.012),
      new PIDController(8.0, 0.0, 0.012),
      new ProfiledPIDController(
        4.0,
        0.0,
        0.012,
        new TrapezoidProfile.Constraints(SCORING_TRAJECTORY_CONFIG.maxAngularVelocity, SCORING_TRAJECTORY_CONFIG.maxAngularAcceleration)
      )
    );
  }

  private static final double CARPET_COEF_FRICTION = 8.0;
  private static final double DRIVE_CURRENT_LIMIT = 400.0;
  public static final double DRIVE_VELOCITY_DEADBAND = 1e-9;
  public static final ChassisSpeeds NON_ZERO_CHASSIS_SPEED = new ChassisSpeeds(1, 1, 0); //TODO should this be smaller?

  public static final double ROBOT_MASS = 60.0;
  public static final double ROBOT_MOI = (2.0 / 12.0) * ROBOT_MASS * (Math.pow(DRIVE_CONFIG.bumperWidthX(), 2));
  public static final RobotConfig SCORING_ROBOT_CONFIG = new RobotConfig(
    ROBOT_MASS,
    ROBOT_MOI,
    new ModuleConfig(
      DRIVE_CONFIG.wheelRadius(),
      SCORING_TRAJECTORY_CONFIG.maxLinearVelocity(),
      CARPET_COEF_FRICTION,
      DCMotor.getKrakenX60(1).withReduction(MODULE_GAINS_AND_RATIOS.driveReduction()),
      DRIVE_CURRENT_LIMIT,
      1
    ),
    MODULE_TRANSLATIONS
  );

  public static final RobotConfig TRAJ_ROBOT_CONFIG = new RobotConfig(
    ROBOT_MASS,
    ROBOT_MOI,
    new ModuleConfig(
      DRIVE_CONFIG.wheelRadius(),
      DRIVE_CONFIG.maxLinearVelocity,
      CARPET_COEF_FRICTION,
      DCMotor.getKrakenX60(1).withReduction(MODULE_GAINS_AND_RATIOS.driveReduction()),
      DRIVE_CURRENT_LIMIT,
      1
    ),
    MODULE_TRANSLATIONS
  );

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
  public record ModuleIDs(
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
