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
package frc.robot.CatzSubsystems.CatzRampPivot;

import frc.robot.CatzConstants;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

/** Add your docs here. */
public class RampPivotConstants {

    public static final boolean isRampPivotDisabled = false;

    public static final int RAMP_PIVOT_MTR_ID = 41;

    public static final double MANUAL_SCALE =  45 * 48 / 14;
    public static final int STALL_CURRENT_LIMIT = 30; //TODO

    public static final double RAMP_STOW = 0.0;
    public static final double RAMP_INTAKE = 10.85;//7.5;//2.240625;//1.536;//2.240625;//4.9; // 0.536

    public static final double RAMP_CLIMB = 34.810546;
    public static final double RAMP_L1_SCORE = 20.0;
    public static final double heightPlaceholder = 10;

    // Initial PIDF and motion magic assignment
    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(2.0, 0.0, 0., 0.0, 0.0, 0.0, 0.0);
            case SN1 -> new Gains(0.015, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(100, 200, 2000);
            case SN_TEST, SN1_2024 -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };
}
