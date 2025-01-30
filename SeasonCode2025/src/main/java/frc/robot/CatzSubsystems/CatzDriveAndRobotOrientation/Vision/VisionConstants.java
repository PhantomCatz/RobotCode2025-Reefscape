// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;

import frc.robot.CatzConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

  public static final VisionIO[] limelights =
      new VisionIO[] {
        // new VisionIOLimeLight("limelight-udon", UDON_TRANSFORM),    //index 0 left
        new VisionIOLimelight("limelight-soba") // index 1 right
        // new VisionIOLimeLight("limelight-ramen", RAMEN_TRANSFORM)    //index 2 turret)
      };

 public static final Pose3d[] cameraPoses =
      switch (CatzConstants.getRobotType()) {
        case SN2 ->
            new Pose3d[] {
              new Pose3d(
                  Units.inchesToMeters(8.875),
                  Units.inchesToMeters(10.5),
                  Units.inchesToMeters(8.25),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)))),
              new Pose3d(
                  Units.inchesToMeters(3.25),
                  Units.inchesToMeters(5.0),
                  Units.inchesToMeters(6.4),
                  new Rotation3d(0.0, Units.degreesToRadians(-16.875), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-4.709)))),
              new Pose3d(
                  Units.inchesToMeters(8.875),
                  Units.inchesToMeters(-10.5),
                  Units.inchesToMeters(8.25),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-30.0)))),
              new Pose3d(
                  Units.inchesToMeters(-16.0),
                  Units.inchesToMeters(-12.0),
                  Units.inchesToMeters(8.5),
                  new Rotation3d(0.0, Units.degreesToRadians(-33.75), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(176.386))))
            };
        case SN1 ->
            new Pose3d[] {
              new Pose3d(
                  Units.inchesToMeters(8.875),
                  Units.inchesToMeters(10.5),
                  Units.inchesToMeters(8.25),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0))))
            };
        default -> new Pose3d[] {};
      };

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        2.0, // Camera 0
        1.0 // Camera 1
      };

  public static final boolean USE_MEGATAG1 = true; // megatag 1 3d solve allows robot to fly

  // Multipliers to apply for MegaTag 2 observations
  public static final double LINEAR_STD_DEV_MEGATAG2_SCALE_FACTOR = 1.0; // More stable than full 3D solve
  public static final double ANGULAR_STD_DEV_MEGATAG2_SCALE_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available
}
