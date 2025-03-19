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
import frc.robot.Utilities.PolynomialRegression;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**************************************************************************************************
 *
 *
 *
 * FeedForwardCharacterization
 *
 *
 *
 **************************************************************************************************/

public class FeedForwardCharacterization extends Command {
  private static final double START_DELAY_SECS = 2.0;
  private static final double RAMP_VOLTS_PER_SEC = 0.1;

  private final FeedForwardCharacterizationData data = new FeedForwardCharacterizationData();
  private final Timer timer = new Timer();
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> velocitySupplier;

  // -----------------------------------------------------------------------------------------------
  //
  // Feed Forward Characterization Constructor
  //
  // -----------------------------------------------------------------------------------------------
  public FeedForwardCharacterization(
      Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> velocitySupplier) {
    addRequirements(subsystem);
    this.voltageConsumer = voltageConsumer;
    this.velocitySupplier = velocitySupplier;
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Initialize
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Execute
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void execute() {
    double time = timer.get();

    // ---------------------------------------------------------------------------------------------
    // Linearly increase the voltage after the start delay and feed it to the consumer
    // Measure and log the resulting velocity
    // ---------------------------------------------------------------------------------------------
    if (time < START_DELAY_SECS) {
      voltageConsumer.accept(0.0);
    } else {
      double voltage = (time - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC;
      voltageConsumer.accept(voltage);
      data.add(velocitySupplier.get(), voltage);
    }
  }

  // -----------------------------------------------------------------------------------------------
  //
  // End
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    timer.stop();
    data.print();
  }

  // -----------------------------------------------------------------------------------------------
  //
  // IsFinished
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public boolean isFinished() {
    return false;
  }

  // ---------------------------------------------------------------------------------------------
  //
  // FeedForwardCharacterization Data Class
  //
  // ---------------------------------------------------------------------------------------------
  public static class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    // ---------------------------------------------------------------------------------------------
    //
    // FeedForwardCharacterization Data Methods
    //
    // ---------------------------------------------------------------------------------------------
    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {
      // ---------------------------------------------------------------------------------------------
      // If the characterization data isn't empty, perform polynomial regression and print the
      // results
      // ---------------------------------------------------------------------------------------------
      if (velocityData.size() == 0 || voltageData.size() == 0) {
        PolynomialRegression regression =
            new PolynomialRegression(
                velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
                voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
                1);

        // System.out.println("FF Characterization Results:");
        // System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
        // System.out.println(String.format("\tR2=%.5f", regression.R2()));
        // System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
        // System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
      }
    }
  } // end of FeedForwardCharacterizationData
}
