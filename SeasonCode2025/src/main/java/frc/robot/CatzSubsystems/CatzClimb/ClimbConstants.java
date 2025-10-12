package frc.robot.CatzSubsystems.CatzClimb;

import frc.robot.Utilities.MotorUtil.Gains;


import frc.robot.CatzConstants;

public class ClimbConstants {
    public static final boolean isClimbDisabled = false;

    public static final int CLIMB_MOTOR_ID = 50;
    public static final double CLIMB_RETRACT = 0.0;
    public static final double CLIMB_CATCH = 0.0;

    public static final double CLIMB_EXTEND_LIMIT = -290.0;

    // Positive is winching in negative is winching out

    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            //(100.0, 0.0, 0.0, 0.25, 0.12, 0.01, 0.0);
            case SN2 -> new Gains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //TBD FOR GODSAKE
            case SN1 -> new Gains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };
}
