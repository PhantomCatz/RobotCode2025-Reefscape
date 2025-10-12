package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.AlgaeRemoverConstants.*;

import org.littletonrobotics.junction.Logger;

public class CatzAlgaeRemover extends SubsystemBase {
  public static final CatzAlgaeRemover Instance = new CatzAlgaeRemover();

  private final AlgaeRemoverIO io;
  private final AlgaeEffectorIOInputsAutoLogged inputs = new AlgaeEffectorIOInputsAutoLogged();

  public boolean algaeRemove = false;

  private CatzAlgaeRemover() {
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
