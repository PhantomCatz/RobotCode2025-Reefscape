// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

public class AlgaePivotConstants {
    public static final boolean isAlgaePivotDisabled = false;

    // Gearbox definitions
    public static final double ALGAE_PIVOT_GEAR_REDUCTION = 6.0;
    public static final double PIVOT_INITIAL_POS = (90.0 / 360.0) * ALGAE_PIVOT_GEAR_REDUCTION;

    public static final double CURRENT_LIMIT = 40.0;
    // Motor ID
    public static final int ALGAE_PIVOT_MOTOR_ID = 25;

    public static final double MANUAL_SCALE = 5;

    // Initial PIDF and motion magic assignment
    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0); //TODO fix gains
            case SN1 -> new Gains(1.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0); //TODO need an kA for robot
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };
    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(100, 200, 800);
            case SN_TEST, SN1_2024 -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };


  public static LoggedTunableNumber tunnablePos = new LoggedTunableNumber("AlgaePivot/TunnablePosition", 1);
  public static LoggedTunableNumber zeroPos  = new LoggedTunableNumber("AlgaePivot/ZeroPos", 90.0);
  public static LoggedTunableNumber slot0_kP = new LoggedTunableNumber("AlgaePivot/kP", 4.0);
  public static LoggedTunableNumber slot0_kI = new LoggedTunableNumber("AlgaePivot/kI", 0.0);
  public static LoggedTunableNumber slot0_kD = new LoggedTunableNumber("AlgaePivot/kD", 0.0006);

  public static LoggedTunableNumber slot0_kS = new LoggedTunableNumber("AlgaePivot/kS", 0);
  public static LoggedTunableNumber slot0_kV = new LoggedTunableNumber("AlgaePivot/kV", 0);
  public static LoggedTunableNumber slot0_kA = new LoggedTunableNumber("AlgaePivot/kA", 0);


}
