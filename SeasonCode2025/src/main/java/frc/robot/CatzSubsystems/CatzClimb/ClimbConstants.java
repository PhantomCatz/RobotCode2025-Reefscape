// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzClimb;

import frc.robot.Utilities.MotorUtil.Gains;


import frc.robot.CatzConstants;

public class ClimbConstants {
    public static final boolean isClimbDisabled = false;

    public static final int CLIMB_MOTOR_ID = 50;

    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            //(100.0, 0.0, 0.0, 0.25, 0.12, 0.01, 0.0);
            case SN2 -> new Gains(12.0, 0.0, 0.3, 0.175, 0.13, 0.013, 0.0);
            case SN1 -> new Gains(12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };
}
