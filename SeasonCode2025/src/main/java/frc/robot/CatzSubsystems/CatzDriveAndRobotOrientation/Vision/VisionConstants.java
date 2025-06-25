package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;


import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

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
  public static final double LINEAR_STD_DEV_BASELINE = 0.04; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.01; // Radians

  public static final VisionIO[] limelights =
      new VisionIO[] {
        // new VisionIOLimeLight("limelight-udon", UDON_TRANSFORM),    //index 0 left
        new VisionIOLimelight("limelight-soba") // index 1 right
        // new VisionIOLimeLight("limelight-ramen", RAMEN_TRANSFORM)    //index 2 turret)
      };

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  public static final Translation2d[] LIMELIGHT_ERROR = {
    new Translation2d(0.07647673800,0.0883352451).div(2.356170511),
    new Translation2d(0.01013154230126, -0.04330684940).div(2.356170511)
    // new Translation2d(-0.0290867, 0.070948).div(1.2907317),
    // new Translation2d(0.04282, 0.0833).div(1.2907317)
  };

  public static final boolean USE_MEGATAG1 = false; // megatag 1 3d solve allows robot to fly

  public static final HashMap<String, Double> PING_MAP = new HashMap<>() {{
    put("limelight-gyoza", 76.25);
    put("limelight-udon", 76.25);

  }};

  // Multipliers to apply for MegaTag 2 observations
  public static final double LINEAR_STD_DEV_MEGATAG2_SCALE_FACTOR = 0.1;//1.0; // More stable than full 3D solve
  public static final double ANGULAR_STD_DEV_MEGATAG2_SCALE_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available
}
