package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.Builder;

public class DriveConstants {
    //-------------------------------------------------------------------------
    // Disabled flag for testing
    //--------------------------------------------------------------------------
    public static final boolean isDriveDisabled = false;

    //---------------------------------------------------------------------------
    // Module organizations
    //---------------------------------------------------------------------------
    public static final String[] moduleNames = new String[] {"FL", "BL", "BR", "FR"};
    public static final int INDEX_FL = 0;
    public static final int INDEX_BL = 1;
    public static final int INDEX_BR = 2;
    public static final int INDEX_FR = 3;

    public static final int GYRO_ID = 0;


    public static final DriveConfig driveConfig =
    switch (CatzConstants.getRobotType()) {
      case SN_TEST, SN2 ->
          DriveConfig.builder()
              .wheelRadius(Units.inchesToMeters(1.8))
              .robotLengthX(Units.inchesToMeters(24.0))
              .robotWidthY(Units.inchesToMeters(23.5))
              .bumperWidthX(Units.inchesToMeters(37))
              .bumperWidthY(Units.inchesToMeters(33))
              .maxLinearVelocity(Units.feetToMeters(17))
              .maxLinearAcceleration(Units.feetToMeters(75.0)) 
              .maxAngularVelocity(Units.degreesToRadians(600)) // Radians
              .maxAngularAcceleration(Units.degreesToRadians(600)) // Radians // TODO verify angle constraints
              .build();
      case SN1 ->
          new DriveConfig(
              Units.inchesToMeters(2.01834634),
              Units.inchesToMeters(20.75),
              Units.inchesToMeters(20.75),
              Units.inchesToMeters(37),
              Units.inchesToMeters(33),
              Units.feetToMeters(12.16),
              Units.feetToMeters(21.32),
              7.93,
              29.89);
    };



    public static final ModuleGainsAndRatios moduleGainsAndRatios =
        switch (CatzConstants.getRobotType()) {
            case SN1 ->
                new ModuleGainsAndRatios(
                    5.0,
                    0.0,
                    1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, // A/(N*m)
                    0.2,
                    0.0,
                    0.7,
                    0.005, 
                    Mk4iReductions.L2_PLUS.reduction,
                    Mk4iReductions.steer.reduction);
            case SN2 ->
                new ModuleGainsAndRatios(
                    5.5,
                    0.6,
                    0.0,
                    1.2,//1.2, //TODO fix to account for non foc
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
        };


    //-------------------------------------------------------------------------------  
    // Odometry Constants
    //--------------------------------------------------------------------------------
    public static final double odometryFrequency =
        switch (CatzConstants.getRobotType()) {
            case SN_TEST -> 50.0;
            case SN1 -> 100.0;
            case SN2 -> 250.0;
        };



    //---------------------------------------------------------------------------------------------------------------------
    // Logged Tunable PIDF values for swerve modules
    //---------------------------------------------------------------------------------------------------------------------
    public static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Drive/Module/DrivekP", moduleGainsAndRatios.drivekP());
    public static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Drive/Module/DrivekD", moduleGainsAndRatios.drivekD());
    public static final LoggedTunableNumber drivekS = new LoggedTunableNumber("Drive/Module/DrivekS", moduleGainsAndRatios.driveFFkS());
    public static final LoggedTunableNumber drivekV = new LoggedTunableNumber("Drive/Module/DrivekV", moduleGainsAndRatios.driveFFkV());
    public static final LoggedTunableNumber steerkP = new LoggedTunableNumber("Drive/Module/steerkP", moduleGainsAndRatios.steerkP());
    public static final LoggedTunableNumber steerkD = new LoggedTunableNumber("Drive/Module/steerkD", moduleGainsAndRatios.steerkD());



    public static final ModuleConfig[] moduleConfigs = 
        switch (CatzConstants.getRobotType()) {
            case SN2 ->
                new ModuleConfig[] {
                    new ModuleConfig(1, 2, 9, 1.4196464857/Math.PI/2+0.5),
                    new ModuleConfig(3, 4, 8, 4.6208462275/Math.PI/2+0.5),
                    new ModuleConfig(5, 6, 7, 0.6691969510/Math.PI/2),
                    new ModuleConfig(7, 8, 6, 2.0568857418/Math.PI/2)
                };
            case SN1 ->
                new ModuleConfig[] {
                    new ModuleConfig(1, 2, 2, -0.22139),
                    new ModuleConfig(3, 4, 1, 0.259),
                    new ModuleConfig(5, 6, 3, 0.188),
                    new ModuleConfig(7, 8, 4, 0.000182)
                };
            case SN_TEST -> 
                new ModuleConfig[] {
                    new ModuleConfig(1, 2, 9, 0.0),
                    new ModuleConfig(3, 4, 8, 0.0),
                    new ModuleConfig(5, 6, 7, 0.0),
                    new ModuleConfig(7, 8, 6, 0.0)
                };
        };
    //-----------------------------------------------------------------------------------------------------------------------------
    //
    //      Drivebase controller/object definements
    //
    //-----------------------------------------------------------------------------------------------------------------------------
    public static final PathConstraints autoPathfindingConstraints = new PathConstraints( // 540 // 720 
                                                                    2.0, driveConfig.maxLinearAcceleration, //max vel causing messup
                                                                    driveConfig.maxAngularVelocity, driveConfig.maxAngularAcceleration);


    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
            new Translation2d( driveConfig.robotLengthX() , driveConfig.robotWidthY()).div(2.0),    //LT FRONT
            new Translation2d(-driveConfig.robotLengthX() , driveConfig.robotWidthY()).div(2.0),    //LT BACK
            new Translation2d(-driveConfig.robotLengthX(), -driveConfig.robotWidthY()).div(2.0),    //RT BACK
            new Translation2d( driveConfig.robotLengthX(), -driveConfig.robotWidthY()).div(2.0)     //RT FRONT
        };    

    // calculates the orientation and speed of individual swerve modules when given
    // the motion of the whole robot
    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(moduleTranslations);

    public static HolonomicDriveController getNewHolController(){
        return new HolonomicDriveController(
            new PIDController(10.0, 0.0, 0.1), 
            new PIDController(10.0, 0.0, 0.1),
            new ProfiledPIDController(
                4, 0, 0,
                new TrapezoidProfile.Constraints(driveConfig.maxAngularVelocity, driveConfig.maxAngularAcceleration)
            )
        );
    }


                                                                            
    /****************************************************************************************
     * 
     * Record and Enum tupes
     * 
     *******************************************************************************************/
    public record ModuleConfig(
        int driveID,
        int steerID,
        int absoluteEncoderChannel,
        double absoluteEncoderOffset) {}

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
        L2     ((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
        L2_16t ((50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0)), // SDS mk4i L2 ratio reduction plus 16 tooth pinion 
        L3     ((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),

        L2_PLUS(6.75 * (14.0 / 16.0)),                        
                                        
        steer((150.0 / 7.0));
    
        final double reduction;
    
        Mk4iReductions(double reduction) {
          this.reduction = reduction;
        }
    }

}