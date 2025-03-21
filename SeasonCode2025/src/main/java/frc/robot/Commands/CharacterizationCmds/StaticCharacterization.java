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
package frc.robot.Commands.CharacterizationCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utilities.LoggedTunableNumber;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**************************************************************************************************
 *
 *
 *
 * StaticCharacterization
 *
 *
 *
 **************************************************************************************************/

public class StaticCharacterization extends Command {
  private static final LoggedTunableNumber currentRampFactor =
      new LoggedTunableNumber("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final LoggedTunableNumber minVelocity =
      new LoggedTunableNumber("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  // -----------------------------------------------------------------------------------------------
  //
  // Static Characterization Constructor
  //
  // -----------------------------------------------------------------------------------------------
  public StaticCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Initialize
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void initialize() {
    timer.restart();
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Execute
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Is Finished
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  // -----------------------------------------------------------------------------------------------
  //
  // End
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + currentInput + " amps");
    inputConsumer.accept(0);
  }
}
