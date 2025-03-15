// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
    public static final double MAXPLANETARY_GEAR_RATIO   = 4.0 * 4.0;
    public static final double ELEVATOR_DRIVING_PULLEY   = 24.0;
    public static final double ELEVATOR_DRIVEN_PULLEY    = 18.0;
    public static final double ELEVATOR_RATIO_STAGE_ONE  = ELEVATOR_DRIVING_PULLEY/ELEVATOR_DRIVEN_PULLEY;
    public static final double FINAL_REDUCATION          = MAXPLANETARY_GEAR_RATIO * ELEVATOR_RATIO_STAGE_ONE;
    public static final double ELEVATOR_SPROCKET_RADIUS  = 0.86; //inches

    // Elevator Limits
    public static final double UPPER_LIMIT_RAD = 162.0;
    public static final double LOWER_LIMIT_RAD = 0.0;

    // Elevator Heights:(Radians)
    public static final double STOW_HEIGHT = 5.0; // Raised to prevent boxtube crashing // /ELEVATOR_SPROCKET_RADIUS;
    public static final double L1_HEIGHT   = 11.0; // 8.6    / ELEVATOR_SPROCKET_RADIUS;
    public static final double L2_HEIGHT   = 33.0; //(25.77-5)/* 29.842*/ / ELEVATOR_SPROCKET_RADIUS; //25.77
    public static final double L3_HEIGHT   = 81.7; //(70.1-8) /*68.8 */  / ELEVATOR_SPROCKET_RADIUS; //70.1
    public static final double L4_HEIGHT   = 149.0; //(141)/* 133.3*/  / ELEVATOR_SPROCKET_RADIUS; //143.12
    public static final double L4_CORAL_ADJ = 158.0;
    public static final double BOT_BOT_ALGAE = 80.6;
    public static final double BOT_TOP_ALGAE = 140.7;
    // Motor ID
    public static final int LEFT_LEADER_ID  = 30;
    public static final int RIGHT_FOLLOWER_ID = 31;

    public static final int BOT_LIMIT_SWITCH = 2;


    public static final double elevatorLength =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> Units.inchesToMeters(24.8);
            default -> Units.inchesToMeters(25.866);
        };

    // Misc constants
    public static final boolean IS_LEADER_INVERTED = false;
    public static final double  MIN_ROTATIONS = 0.0;
    public static final double  MAX_ROTATIONS = 117.0;
    public static final Translation2d elevatorOrigin = new Translation2d(-0.238, 0.298);
    public static final double MANUAL_SCALE = 0.5;
    // Initial PIDF and motion magic assignment
    public static final Gains slot0_gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(125.0, 0.0, 0.0, 0.175, 0.130, 0.013, 0.4);
            case SN1 -> new Gains(75.0, 0.0, 0.0, 0.175, 0.3, 0.13, 0.4); //
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final Gains slot1_gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(125.0, 0.0, 0.0, 0.175, 0.130, 0.013, 0.4);
            case SN1 -> new Gains(75.0, 0.1, 0.0, 0.175, 0.13, 0.013, 0.4); //
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };
    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(100, 200, 2000);
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
    
    public static final LoggedTunableNumber mmCruiseVelocity = new LoggedTunableNumber("Elevator/Gains/kV", motionMagicParameters.mmCruiseVelocity());
    public static final LoggedTunableNumber mmAcceleration = new LoggedTunableNumber("Elevator/Gains/kA", motionMagicParameters.mmAcceleration());
    public static final LoggedTunableNumber mmJerk = new LoggedTunableNumber("Elevator/Gains/kG", motionMagicParameters.mmJerk());
    public static final LoggedTunableNumber lowerLimitRotations = new LoggedTunableNumber("Elevator/LowerLimitDegrees", MIN_ROTATIONS);
    public static final LoggedTunableNumber upperLimitRotations = new LoggedTunableNumber("Elevator/UpperLimitDegrees", MAX_ROTATIONS);

}
