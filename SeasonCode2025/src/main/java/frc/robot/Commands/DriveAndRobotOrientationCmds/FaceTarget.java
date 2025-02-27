// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.DriveAndRobotOrientationCmds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.Utilities.AllianceFlipUtil;

/**************************************************************************************************
 *
 *
 *
 * FaceTarget
 *
 *
 *
 **************************************************************************************************/

public class FaceTarget extends Command {
  private final double kP = 0.1;
  private final double MIN_ERROR = 2; // degrees
  private final double TIMEOUT = 3;

  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
  private CatzDrivetrain drivetrain;

  private Rotation2d rotDif = new Rotation2d();
  private Translation2d target;
  private Translation2d useTarget;
  private Timer timer = new Timer();

  // -----------------------------------------------------------------------------------------------
  //
  // Face Target Command Constructor
  //
  // -----------------------------------------------------------------------------------------------
  public FaceTarget(Translation2d target, CatzDrivetrain drivetrain) {
    this.target = target;
    this.drivetrain = drivetrain;
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Initialize
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void initialize() {
    useTarget = target;
    if (AllianceFlipUtil.shouldFlipToRed()) {
      useTarget = AllianceFlipUtil.apply(target);
    }
    timer.reset();
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Execute
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void execute() {
    Pose2d curPose = tracker.getEstimatedPose();
    Translation2d posDif = useTarget.minus(curPose.getTranslation());
    rotDif =
        Rotation2d.fromRadians(Math.atan2(posDif.getY(), posDif.getX()))
            .minus(curPose.getRotation());

    drivetrain.drive(new ChassisSpeeds(0, 0, rotDif.getDegrees() * kP));
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Is Finished
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public boolean isFinished() {
    return Math.abs(rotDif.getDegrees()) < MIN_ERROR || timer.get() > TIMEOUT;
  }
}
