// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.AlgaeRemoverConstants.*;

import org.littletonrobotics.junction.Logger;

public class CatzAlgaeRemover extends SubsystemBase {

  private final AlgaeRemoverIO io;
  private final AlgaeEffectorIOInputsAutoLogged inputs = new AlgaeEffectorIOInputsAutoLogged();



  public CatzAlgaeRemover() {
    if(isAlgaeRemoverDisabled) { //Comes from Algae Remover Constants
      io = new AlgaeRemoverIONull();
      System.out.println("Algae Remover Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new AlgaeRemoverIOReal();
          System.out.println("Algae Remover Configured for Real");
        break;
        case REPLAY:
          io = new AlgaeRemoverIOReal() {};
          System.out.println("Algae Remover Configured for Replayed simulation");
        break;
        default:
          io = new AlgaeRemoverIONull();
          System.out.println("Algae Remover Unconfigured");
        break;
      }
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
    return runOnce(() -> io.runMotor(ALGAE_REMOVER_POWER));
  }

  public Command vomitAlgae() {
    return runOnce(() -> io.runMotor(-ALGAE_REMOVER_POWER));
  }

  public Command runMotorTop(double spd) {
    return run(() -> runMotorTop(spd));
  }

  public Command runMotorBottom(double spd) {
    return run(() -> runMotorBottom(spd));
  }

  public Command stopAlgae() {
    return runOnce(() -> io.runMotor(0));
  }
}
