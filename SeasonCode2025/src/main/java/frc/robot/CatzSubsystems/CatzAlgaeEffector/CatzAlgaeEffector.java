// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import org.littletonrobotics.junction.Logger;

public class CatzAlgaeEffector extends SubsystemBase {

  private final AlgaeEffectorIO io;
  private final AlgaeEffectorIOInputsAutoLogged inputs = new AlgaeEffectorIOInputsAutoLogged();

  // private static LoggedTunableNumber tunableNumber = new LoggedTunableNumber("Intake/MotorPower",
  // 0.5);

  // private outtakeStates previousState = outtakeStates.STOP;

  public CatzAlgaeEffector() {
    switch (CatzConstants.hardwareMode) {
      case REAL:
        io = new AlgaeEffectorIOSparkmax();
        break;
      case REPLAY:
        io = new AlgaeEffectorIOSparkmax() {};
        break;
      default:
        io = new AlgaeEffectorIOSparkmax();
        break;
    }
  }

  double algaeEffectorBottom = 0.5;
  double algaeEffectorTop = 0.5;
  double adj_speed = 0.2;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("effector", inputs);
  }

  public Command runMotor() {
    return run(() -> io.runMotor(algaeEffectorTop, algaeEffectorBottom));
  }

  public Command runMotorBck(double spd) {
    return run(() -> io.runMotorBck(spd));
  }

  public Command runMotorTop(double spd) {
    return run(() -> runMotorTop(spd));
  }

  public Command runMotorBottom(double spd) {
    return run(() -> runMotorBottom(spd));
  }


  // ============================================
  //
  //              Case methods
  //
  // ============================================
}
