package frc.robot.Commands.CharacterizationCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utilities.LoggedTunableNumber;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**************************************************************************************************

* 

* StaticCharacterization

* 
 
**************************************************************************************************/

public class StaticCharacterization extends Command {
  private static final LoggedTunableNumber currentRampFactor = new LoggedTunableNumber("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final LoggedTunableNumber minVelocity = new LoggedTunableNumber("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  //-----------------------------------------------------------------------------------------------
  //
  // Static Characterization Constructor
  //
  //-----------------------------------------------------------------------------------------------
  public StaticCharacterization(Subsystem subsystem, DoubleConsumer characterizationInputConsumer, DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  //-----------------------------------------------------------------------------------------------
  //
  // Initialize 
  //
  //-----------------------------------------------------------------------------------------------
  @Override
  public void initialize() {
    timer.restart();
  }

  //-----------------------------------------------------------------------------------------------
  //
  // Execute 
  //
  //-----------------------------------------------------------------------------------------------
  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
  }

  //-----------------------------------------------------------------------------------------------
  //
  // Is Finished 
  //
  //-----------------------------------------------------------------------------------------------
  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  //-----------------------------------------------------------------------------------------------
  //
  // End 
  //
  //-----------------------------------------------------------------------------------------------
  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + currentInput + " amps");
    inputConsumer.accept(0);
  }
}
