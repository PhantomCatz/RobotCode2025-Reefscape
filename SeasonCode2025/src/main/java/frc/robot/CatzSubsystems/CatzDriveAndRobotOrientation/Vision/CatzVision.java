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
package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.TxTyObservation;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.VisionIO.PoseObservation;
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

  private PoseObservation[] poseObservations;

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
  public double getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservations[0].tx();
  }

  public PoseObservation[] getPoseObservation() {
    return poseObservations;
  }

  public int getTagId(int cameraIndex) {
    if (inputs[cameraIndex].latestTargetObservations.length > 0) {
      return (int) inputs[cameraIndex].latestTargetObservations[0].tagID();
    }
    return -1;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("RealInputs/Vision/Camera" + i, inputs[i]);
    }
    poseObservations = inputs[0].poseObservations;

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>();
    Map<Integer, TxTyObservation> txTyObservations = new HashMap<>();
    Pose2d robotPose = CatzRobotTracker.getInstance().getEstimatedPose();

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
        if(inputs[cameraIndex].tagIds.length == 0){
          continue;
        }

        int tagId = inputs[cameraIndex].tagIds[0];
        Pose3d observationPose = observation.pose();
        Pose2d apriltagPose = FieldConstants.APRILTAG_LAYOUT.getTagPose(tagId).get().toPose2d();

        Translation2d limelightError = VisionConstants.LIMELIGHT_ERROR[cameraIndex].times(apriltagPose.getTranslation().minus(robotPose.getTranslation()).getNorm()).rotateBy(apriltagPose.getRotation());
        observationPose.plus(new Transform3d(limelightError.getX(), limelightError.getY(), 0.0, new Rotation3d()));

        boolean rejectPose =
            ((observation.tagCount() == 0) // Must have at least one tag
                // || (observation.tagCount() == 1  && observation.ambiguity() > maxAmbiguity) //
                // Cannot be high ambiguity // TODO add back in
                // || inputs[cameraIndex].ta < 2 //TODO add scalar for distance to the ta for standard devs
                || (Math.abs(observationPose.getZ())
                    >= maxZError) // Must have realistic Z coordinate
                // Must be within the field boundaries
                || (observationPose.getX() < 0.0)
                || (observationPose.getX() > aprilTagLayout.getFieldLength())
                || (observationPose.getY() < 0.0)
                || (observationPose.getY() > aprilTagLayout.getFieldWidth()))
                // Filter out megatag 1 observations
                || (!VisionConstants.USE_MEGATAG1 && observation.type() == PoseObservationType.MEGATAG_1);
        // Add pose to log
        robotPoses.add(observationPose);
        if (rejectPose) {
          robotPosesRejected.add(observationPose);
        } else {
          robotPosesAccepted.add(observationPose);
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        //------------------------------------------------------------------------------------------------------------------------------------
        // Calculate standard deviations
        //------------------------------------------------------------------------------------------------------------------------------------
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount(); //TODO tune
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
                    inputs[cameraIndex].name,
                    observationPose.toPose2d(),
                    observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));

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
      //------------------------------------------------------------------------------------------------------------------------------------
      // Get tag tx ty observation data
      for (int frameIndex = 0; frameIndex < inputs[cameraIndex].latestTargetObservations.length;
          frameIndex++) {
        var timestamp = inputs[cameraIndex].latestTargetObservations[frameIndex].timestamp();
        var values = inputs[cameraIndex].latestTargetObservations[frameIndex];

        double tx = values.tx();
        double ty = values.ty();
        int tagId = (int) values.tagID();
        double distance = values.distance();

        txTyObservations.put(
            tagId, new TxTyObservation(tagId, cameraIndex, tx, ty, distance, timestamp));

        Logger.recordOutput("Vision/Camera" + cameraIndex + "/TxTyObservation tx" + frameIndex, tx);
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/TxTyObservation ty" + frameIndex, ty);
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/TxTyObservation tagID" + frameIndex, tagId);
        Logger.recordOutput("Vision/Camera" + cameraIndex + "/TxTyObservation distance" + frameIndex, distance);
      }

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

  public boolean isSeeingApriltag(){
    for(int i = 0; i < inputs.length; i++){
      if(inputs[i].tagIds.length > 0){
        return true;
      }
    }
    System.out.println("NOT WEEING ANYTHING");
    return false;
  }
}
