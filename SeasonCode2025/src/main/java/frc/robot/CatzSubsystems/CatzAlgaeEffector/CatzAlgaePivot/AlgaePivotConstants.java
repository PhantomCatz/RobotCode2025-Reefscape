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
package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

public class AlgaePivotConstants {
    public static final boolean isAlgaePivotDisabled = true;

    // Gearbox definitions
    public static final double ALGAE_PIVOT_GEAR_REDUCTION = 6.0;
    public static final double PIVOT_INITIAL_POS = (95.0 / 360.0) * ALGAE_PIVOT_GEAR_REDUCTION;

    public static final double CURRENT_LIMIT = 40.0;
    // Motor ID
    public static final int ALGAE_PIVOT_MOTOR_ID = 25;

    public static final double MANUAL_SCALE = 0.5;

    // Initial PIDF and motion magic assignment
    public static final Gains slot0_gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            case SN1 -> new Gains(10.0, 0.0, 0.1, 0.00, 0.0, 0.0, 3.0); //kg1.5
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final Gains slot1_gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(2.0, 0.0, 0.0, 0.175, 0.130, 0.013, 0.4);
            case SN1 -> new Gains(1.0, 0.1, 0.0, 0.175, 0.13, 0.013, 0.4); //
            case SN_TEST, SN1_2024 -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.2, 0.2, 22.9);
        };

    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(100.0, 200.0, 800.0);
            case SN_TEST, SN1_2024 -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };


  public static LoggedTunableNumber tunnablePos = new LoggedTunableNumber("AlgaePivot/tunnable Pos", 1);
  public static LoggedTunableNumber zeroPos  = new LoggedTunableNumber("AlgaePivot/ZeroPos", 90.0);

  public static LoggedTunableNumber slot0_kP = new LoggedTunableNumber("AlgaePivot/slot_0 kP", 4.0);
  public static LoggedTunableNumber slot0_kI = new LoggedTunableNumber("AlgaePivot/slot_0 kI", 0.0);
  public static LoggedTunableNumber slot0_kD = new LoggedTunableNumber("AlgaePivot/slot_0 kD", 0.0);
  public static LoggedTunableNumber slot0_kS = new LoggedTunableNumber("AlgaePivot/slot_0 kS", 0.0);
  public static LoggedTunableNumber slot0_kV = new LoggedTunableNumber("AlgaePivot/slot_0 kV", 0.0);
  public static LoggedTunableNumber slot0_kA = new LoggedTunableNumber("AlgaePivot/slot_0 kA", 0.0);

  public static LoggedTunableNumber slot1_kP = new LoggedTunableNumber("AlgaePivot/slot_1 kP", 4.0);
  public static LoggedTunableNumber slot1_kI = new LoggedTunableNumber("AlgaePivot/slot_1 kI", 0.0);
  public static LoggedTunableNumber slot1_kD = new LoggedTunableNumber("AlgaePivot/slot_1 kD", 0.0);
  public static LoggedTunableNumber slot1_kS = new LoggedTunableNumber("AlgaePivot/slot_1 kS", 0.0);
  public static LoggedTunableNumber slot1_kV = new LoggedTunableNumber("AlgaePivot/slot_1 kV", 0.0);
  public static LoggedTunableNumber slot1_kA = new LoggedTunableNumber("AlgaePivot/slot_1 kA", 0.0);


}
