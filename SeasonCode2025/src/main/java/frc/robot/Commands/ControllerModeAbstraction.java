// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControllerModeAbstraction {

  private static boolean m_isModeSpeaker;

  public static boolean isModeSpeaker() {
    return m_isModeSpeaker;
  }

  public static Command sortModes(boolean isModeSpeaker) {
    return Commands.runOnce(() -> m_isModeSpeaker = isModeSpeaker);
  }

  // public static Command cancelController(RobotContainer container) {
  //     return container.getCatzSuperstructure().cancelSuperStructureCommands();
  // }

  public static void periodicDebug() {}
}
