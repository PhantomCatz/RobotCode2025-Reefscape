// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import frc.robot.CatzConstants;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

public class AlgaePivotConstants {
    public static final boolean isAlgaePivotDisabled = true;

    // Gearbox definitions

    // Motor ID
    public static final int ALGAE_PIVOT_MOTOR_ID = 40;

    // Initial PIDF and motion magic assignment
    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //TODO fix gains
            case SN1 -> new Gains(4.0, 0.0, 0.0, 0.25, 0.12, 0.0, 0.0); //TODO need an kA for robot
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };
    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(400, 800, 1600);
            case SN_TEST, SN1_2024 -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };


}
