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
package frc.robot.Utilities;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

/**
 * A command that does nothing but ends after a specified match time or condition. Useful for
 * CommandGroups.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class waituntil extends Command {
  private final BooleanSupplier m_condition;

  /**
   * Creates a new WaitUntilCommand that ends after a given condition becomes true.
   *
   * @param condition the condition to determine when to end
   */
  public waituntil(BooleanSupplier condition) {
    m_condition = requireNonNullParam(condition, "condition", "WaitUntilCommand");
  }

  /**
   * Creates a new WaitUntilCommand that ends after a given match time.
   *
   * <p>NOTE: The match timer used for this command is UNOFFICIAL. Using this command does NOT
   * guarantee that the time at which the action is performed will be judged to be legal by the
   * referees. When in doubt, add a safety factor or time the action manually.
   *
   * @param time the match time after which to end, in seconds
   */
  public waituntil(double time) {
    this(() -> Timer.getMatchTime() - time > 0);
  }

  @Override
  public boolean isFinished() {
    System.out.println("isfins?" + m_condition.getAsBoolean());
    return m_condition.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted){
    System.out.println("donee!");
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
