// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector;
import static frc.robot.CatzSubsystems.CatzAlgaeEffector.AlgaeRemoverConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import org.littletonrobotics.junction.Logger;

public class CatzAlgaeRemover extends SubsystemBase {

  private final AlgaeRemoverIO io;
  private final AlgaeEffectorIOInputsAutoLogged inputs = new AlgaeEffectorIOInputsAutoLogged();

  public CatzAlgaeRemover() {
    switch (CatzConstants.hardwareMode) {
      case REAL:
        io = new AlgaeRemoverIOReal();
        break;
      case REPLAY:
        io = new AlgaeRemoverIOReal() {};
        break;
      default:
        io = new AlgaeRemoverIONull();
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeRemover/inputs", inputs);
  }

  //=====================================================================================
  //
  //              Processor Commands
  //
  //=====================================================================================
  public Command eatAlgae() {
    return runOnce(() -> io.runMotor(ALGAE_EFFECTOR_TOP, ALGAE_EFFECTOR_BOT));
  }

  public Command vomitAlgae(double spd) {
    return runOnce(() -> io.runMotor(-ALGAE_EFFECTOR_TOP, -ALGAE_EFFECTOR_BOT));
  }

  public Command runMotorTop(double spd) {
    return run(() -> runMotorTop(spd));
  }

  public Command runMotorBottom(double spd) {
    return run(() -> runMotorBottom(spd));
  }

}
