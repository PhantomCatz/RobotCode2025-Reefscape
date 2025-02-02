// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzOuttake;

/** Add your docs here. */
public class OuttakeConstants {

    // Subsystem safety disable
    public static final boolean isOuttakeDisabled = true;

    public static final int OUTTAKE_CURRENT_LIMIT = 30;

    public static final int LEFT_OUTTAKE_ID = 21;
    public static final int RIGHT_OUTTAKE_ID = 20;
    public static final int FRONT_BEAM_BREAK_ID = 1;
    public static final int BACK_BEAM_BREAK_ID = 0;

    static double OUTTAKE_RT = 0.3;
    static double OUTTAKE_LT = 0.3;
    static double INTAKE_SPD = 0.3;
    static double ADJ_SPD = 0.1;
}
