// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.FieldConstants;
import frc.robot.Utilities.AllianceFlipUtil;

public class WaitUntilPassX extends Command {
  private double xCoord;
  private double originalXCoord;
  private Command cmd;
  private boolean executing = false;
  private boolean done = false;

  private final double ERROR_RANGE = 0.2; // im pretty sure this is 20cm

  public WaitUntilPassX(double originalXCoord, Command command) {
    this.originalXCoord = originalXCoord;
    this.cmd = command;
  }

  @Override
  public void initialize() {
    executing = false;
    done = false;
    if (AllianceFlipUtil.shouldFlipToRed()) {
      xCoord = FieldConstants.FIELD_LENGTH_MTRS - originalXCoord;
    } else {
      xCoord = originalXCoord;
    }
  }

  @Override
  public void execute() {
    if (Math.abs(xCoord - CatzRobotTracker.getInstance().getEstimatedPose().getX())
        <= ERROR_RANGE) {
      executing = true;
      cmd.initialize();
    }

    if (executing) {
      cmd.execute();
      if (cmd.isFinished()) {
        done = true;
        cmd.end(false);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
