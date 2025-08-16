package frc.robot.CatzSubsystems.CatzOuttake;

import frc.robot.CatzConstants;
import frc.robot.Utilities.MotorUtil.Gains;

/** Add your docs here. */
public class OuttakeConstants {

    // Subsystem safety disable
    public static final boolean isOuttakeDisabled = true;

    public static final int OUTTAKE_CURRENT_LIMIT = 30;
    public static final int LEFT_OUTTAKE_ID = 21;
    public static final int RIGHT_OUTTAKE_ID = 20;
    public static final int FRONT_BEAM_BREAK_ID = 1;
    public static final int BACK_BEAM_BREAK_ID = 0;

    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //TODO fix gains
            case SN1 -> new Gains(75.0, 0.0, 0.0, 0.175, 0.13, 0.013, 0.4); //
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    static double OUTTAKE_RT = 0.4;
    static double OUTTAKE_LT = 0.4;
    static double OUTTAKE_L4 = 0.24;
    static double INTAKE_SPD = 0.4;
    static double ADJ_SPD = 0.1;
    static double OUTTAKE_L1_RT = 0.05;
    static double OUTTAKE_L1_LT = 0.25;
}
