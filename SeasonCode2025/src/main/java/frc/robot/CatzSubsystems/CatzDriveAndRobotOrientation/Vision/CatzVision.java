// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.TxTyObservation;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.VisionIO.PoseObservationType;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

public class CatzVision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public CatzVision(VisionIO... io) {
    this.io = io;
    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("inputs/Vision/Camera" + i, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>();
    Map<Integer, TxTyObservation> txTyObservations = new HashMap<>();

    //------------------------------------------------------------------------------------------------------------------------------------
    // Get Global Pose observation data
    //------------------------------------------------------------------------------------------------------------------------------------
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      //------------------------------------------------------------------------------------------------------------------------------------
      // Loop over pose observations for megatag 1 and 2
      //------------------------------------------------------------------------------------------------------------------------------------
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            ((observation.tagCount() == 0) // Must have at least one tag
                // || (observation.tagCount() == 1  && observation.ambiguity() > maxAmbiguity) //
                // Cannot be high ambiguity // TODO add back in
                || (Math.abs(observation.pose().getZ())
                    >= maxZError) // Must have realistic Z coordinate
                // Must be within the field boundaries
                || (observation.pose().getX() < 0.0)
                || (observation.pose().getX() > aprilTagLayout.getFieldLength())
                || (observation.pose().getY() < 0.0)
                || (observation.pose().getY() > aprilTagLayout.getFieldWidth()));
        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        //------------------------------------------------------------------------------------------------------------------------------------
        // Calculate standard deviations
        //------------------------------------------------------------------------------------------------------------------------------------
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
        double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= LINEAR_STD_DEV_MEGATAG2_SCALE_FACTOR;
          angularStdDev *= ANGULAR_STD_DEV_MEGATAG2_SCALE_FACTOR;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        CatzRobotTracker.getInstance()
            .addVisionObservation(
                new VisionObservation(
                    observation.pose().toPose2d(),
                    observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
        // System.out.println("===");

        // Log camera datadata
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
      }

      //------------------------------------------------------------------------------------------------------------------------------------
      // Get tag tx ty observation data
      //------------------------------------------------------------------------------------------------------------------------------------;
      var targetObservation = inputs[cameraIndex].latestTargetObservation;

      
      double distance = 0.1;
      txTyObservations.put(
            targetObservation.tagID(), new TxTyObservation(targetObservation.tagID(), 
                                                           cameraIndex, 
                                                           targetObservation.tx().getDegrees(), 
                                                           targetObservation.ty().getDegrees(), 
                                                           distance, 
                                                           targetObservation.timestampe()));
    

      // Save tx ty observation data
      for (var observation : txTyObservations.values()) {
        if (!allTxTyObservations.containsKey(observation.tagId())
            || observation.distance() < allTxTyObservations.get(observation.tagId()).distance()) {
          allTxTyObservations.put(observation.tagId(), observation);
        }
      }

      allTxTyObservations.values().stream().forEach(CatzRobotTracker.getInstance()::addTxTyObservation);


    }

  }
}
