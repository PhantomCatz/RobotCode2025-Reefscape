package frc.robot;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.GeomUtil;

/**
   * Contains various field dimensions and useful reference points. Dimensions are
   * in meters, and sets
   * of corners start in the lower left moving clockwise. <b>All units in
   * Meters</b> <br>
   * <br>
   *
   * <p>
   * All translations and poses are stored with the origin at the rightmost point
   * on the BLUE
   * ALLIANCE wall.<br>
   * <br>
   * Length refers to the <i>x</i> direction (as described by wpilib) <br>
   * Width refers to the <i>y</i> direction (as described by wpilib)
   */
  public class FieldConstants {
   public static final double FIELD_LENGTH_MTRS = Units.inchesToMeters(651.223);

  
  }