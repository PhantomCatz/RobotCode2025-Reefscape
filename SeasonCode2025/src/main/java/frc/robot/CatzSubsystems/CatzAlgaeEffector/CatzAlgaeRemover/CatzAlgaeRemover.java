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
    Logger.processInputs("RealInputs/AlgaeRemover", inputs);
  }

  //=====================================================================================
  //
  //              Processor Commands
  //
  //=====================================================================================
  public Command eatAlgae() {
    return runOnce(() -> io.runPercentOutput(ALGAE_EAT));
  }

  public Command holdAlgae() {
    return runOnce(() -> io.runPercentOutput(ALGAE_HOLD));
  }

  public Command vomitAlgae() {
    return runOnce(() -> io.runPercentOutput(ALGAE_VOMIT));
  }

  public Command stopAlgae() {
    return runOnce(() -> io.runPercentOutput(0));
  }
}
