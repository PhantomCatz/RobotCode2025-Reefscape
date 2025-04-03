//------------------------------------------------------------------------------------
// 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project. 
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems.CatzElevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

/** Add your docs here. */
public class ElevatorConstants {
    public static final boolean isElevatorDisabled = false;

    // Gearbox definitions
    public static final double ELEVATOR_GEAR_RATIO   = (( 42.0 / 12.0) * (22.0 / 16.0));
    public static final double T16_DIAMETER_INCHES       = 1.503;
    //For every 9 rotations of the motor, the output shaft rotates once.
    //For every inch moved up by by the sprocket, the carriage rises by 2 inches
    public static final double FINAL_RATIO               = 2 * (T16_DIAMETER_INCHES * Math.PI) / ELEVATOR_GEAR_RATIO;

    //inches TODO tune
    public static final double STOW_HEIGHT = 1.0;
    public static final double L1_HEIGHT   = 5.6;
    public static final double L2_HEIGHT   = 16.122;
    public static final double L3_HEIGHT   = 34.763;
    public static final double L4_HEIGHT   = 65.443;
    public static final double L4_CORAL_ADJ = 69.0;
    public static final double BOT_BOT_ALGAE = 20.6;
    public static final double BOT_TOP_ALGAE = 40.7;
    // Motor ID
    public static final int LEFT_LEADER_ID  = 31;
    public static final int RIGHT_FOLLOWER_ID = 30;

    public static final int BOT_LIMIT_SWITCH = 2;


    public static final double elevatorLength =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> Units.inchesToMeters(24.8);
            default -> Units.inchesToMeters(25.866);
        };

    // Misc constants
    public static final boolean IS_LEADER_INVERTED = false;
    public static final double  MIN_TRAVEL_INCHES = 0.0;
    public static final double  MAX_TRAVEL_INCHES = 164.0;
    public static final Translation2d elevatorOrigin = new Translation2d(-0.238, 0.298);
    public static final double MANUAL_SCALE = 0.5;

    // Initial PIDF and motion magic assignment
    public static final Gains slot0_gains =
        switch (CatzConstants.getRobotType()) {
            //case SN2 -> new Gains(8.0, 0.0, 0.0, 0.175, 0.230, 0.013, 0.4);
            case SN2 -> new Gains(4.0, 0.0, 0.0, 0.065, 0.379, 0.009, 0.0);//            case SN2 -> new Gains(4.0, 0.0, 0.0, 0.175, 0.425, 0.022, 0.0);

            case SN1 -> new Gains(3.0, 0.0, 0.0, 0.175, 0.3, 0.013, 0.4); //

            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final Gains slot1_gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(3.0, 0.0, 0.0, 0.175, 0.130, 0.013, 0.4);
                               // v 75.0 vv 0.1 v
            case SN1 -> new Gains(3.0, 0.0, 0.0, 0.175, 0.13, 0.013, 0.4); //TODO
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(150, 250, 2000);

            case SN_TEST, SN1_2024 -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };

    // Adjustable Dashboard PIDF values
    public static final LoggedTunableNumber slot0_kP = new LoggedTunableNumber("Elevator/Gains/slot0_kP", slot0_gains.kP());
    public static final LoggedTunableNumber slot0_kI = new LoggedTunableNumber("Elevator/Gains/slot0_kI", slot0_gains.kI());
    public static final LoggedTunableNumber slot0_kD = new LoggedTunableNumber("Elevator/Gains/slot0_kD", slot0_gains.kD());
    public static final LoggedTunableNumber slot0_kS = new LoggedTunableNumber("Elevator/Gains/slot0_kS", slot0_gains.kS());
    public static final LoggedTunableNumber slot0_kV = new LoggedTunableNumber("Elevator/Gains/slot0_kV", slot0_gains.kV());
    public static final LoggedTunableNumber slot0_kA = new LoggedTunableNumber("Elevator/Gains/slot0_kA", slot0_gains.kA());
    public static final LoggedTunableNumber slot0_kG = new LoggedTunableNumber("Elevator/Gains/slot0_kG", slot0_gains.kG());

    public static final LoggedTunableNumber slot1_kP = new LoggedTunableNumber("Elevator/Gains/slot1_kP", slot1_gains.kP());
    public static final LoggedTunableNumber slot1_kI = new LoggedTunableNumber("Elevator/Gains/slot1_kI", slot1_gains.kI());
    public static final LoggedTunableNumber slot1_kD = new LoggedTunableNumber("Elevator/Gains/slot1_kD", slot1_gains.kD());
    public static final LoggedTunableNumber slot1_kS = new LoggedTunableNumber("Elevator/Gains/slot1_kS", slot1_gains.kS());
    public static final LoggedTunableNumber slot1_kV = new LoggedTunableNumber("Elevator/Gains/slot1_kV", slot1_gains.kV());
    public static final LoggedTunableNumber slot1_kA = new LoggedTunableNumber("Elevator/Gains/slot1_kA", slot1_gains.kA());
    public static final LoggedTunableNumber slot1_kG = new LoggedTunableNumber("Elevator/Gains/slot1_kG", slot1_gains.kG());

    public static final LoggedTunableNumber mmCruiseVelocity = new LoggedTunableNumber("Elevator/Gains/Magic Cruise Vel", motionMagicParameters.mmCruiseVelocity());
    public static final LoggedTunableNumber mmAcceleration = new LoggedTunableNumber("Elevator/Gains/Magic Accerlation", motionMagicParameters.mmAcceleration());
    public static final LoggedTunableNumber mmJerk = new LoggedTunableNumber("Elevator/Gains/Magic Jerk", motionMagicParameters.mmJerk());
    public static final LoggedTunableNumber lowerLimitRotations = new LoggedTunableNumber("Elevator/LowerLimitInches", MIN_TRAVEL_INCHES);
    public static final LoggedTunableNumber upperLimitRotations = new LoggedTunableNumber("Elevator/UpperLimitInches", MAX_TRAVEL_INCHES);

}
