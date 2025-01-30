// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  public static enum PathPlannerFlippingState {
    FLIPPED_TO_RED,
    BLUE
  }

  public static PathPlannerFlippingState flippingState;
  public static boolean isPathPlannerFlippedRed = false;

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlipToRed()) {
      return new Translation2d(
        FieldConstants.FIELD_LENGTH_MTRS - translation.getX(),
        FieldConstants.FIELD_WIDTH - translation.getY()
      );
    } else {
      return translation;
    }
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlipToRed()) {
      return new Translation3d(
        FieldConstants.FIELD_LENGTH_MTRS - translation3d.getX(),
        FieldConstants.FIELD_WIDTH - translation3d.getY(),
        translation3d.getZ()
      );
    } else {
      return translation3d;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlipToRed()) {
      return rotation.rotateBy(Rotation2d.k180deg);
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlipToRed()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  public static boolean shouldFlipToRed() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
