// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.VisionConstants.SOBA_TRANSFORM;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.VisionConstants.UDON_TRANSFORM;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.CatzConstants;


/** Add your docs here. */
public class VisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.005;
  public static final double thetaStdDevCoefficient = 0.01;

  public static final int LIMELIGHT_PIPLINE_APRILTAG = 0;
  public static final int LIMELIGHT_PIPLINE_NEURALNETWORK_NOTE = 1;
  public static final int LIMELIGHT_PIPLINE_NEURALNETWORK_YELLOW = 2;


  public static final Transform3d UDON_TRANSFORM = new Transform3d(0.0, 
                                                                    0.0, 
                                                                    0.0, 
                                                                    new Rotation3d(
                                                                      0.0, 0.0, 0.
                                                                    )
                                                                  );

  public static final Transform3d SOBA_TRANSFORM = new Transform3d(0.0, 
                                                                   0.0, 
                                                                   0.0, 
                                                                    new Rotation3d(
                                                                      0.0, 0.0, 0.
                                                                    )
                                                    );

  public static final Transform3d RAMEN_TRANSFORM = new Transform3d(0.0, 
                                                                    0.0, 
                                                                    0.0, 
                                                                    new Rotation3d(
                                                                      0.0, 0.0, 0.
                                                                    )
                                                                  );

  public static final VisionIO[] limelights = new VisionIO[] {
    new VisionIOLimeLight("limelight-udon", UDON_TRANSFORM),    //index 0 left
    new VisionIOLimeLight("limelight-soba", SOBA_TRANSFORM),    //index 1 right
    new VisionIOLimeLight("limelight-ramen", RAMEN_TRANSFORM)    //index 2 turret)
  };

  public static final Transform3d[] limelightTransform = new Transform3d[] {
    UDON_TRANSFORM,
    SOBA_TRANSFORM,
    RAMEN_TRANSFORM
    
  };

  public static final double[] stdDevFactors =
      switch (CatzConstants.getRobotType()) {
        case SN2 -> new double[] {1.0, 0.6, 1.0};
        case SN1 -> new double[] {1.0, 1.0};
        default -> new double[] {};
      };

}
