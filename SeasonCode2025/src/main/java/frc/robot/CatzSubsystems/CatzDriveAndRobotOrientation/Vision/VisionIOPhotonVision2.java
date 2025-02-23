// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

import static frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.VisionConstants.aprilTagLayout;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision2 implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final Transform3d cameraToRobot;
  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();

  private Transform3d rotate(Rotation3d rot){
    return new Transform3d(0, 0, 0, rot);
  }

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision2(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.cameraToRobot = robotToCamera.inverse();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    double timestamp = Timer.getFPGATimestamp();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {

      // Update latest target observation
      List<TargetObservation> targetObservations = new LinkedList<>();
      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        int tagId = target.fiducialId;

        // Calculate robot pose3d
        Pose3d tagPose = aprilTagLayout.getTagPose(tagId).get();
        Pose2d robotPose = tracker.getEstimatedPose();
        tagPose = new Pose3d(tagPose.getTranslation(), new Rotation3d(tagPose.getRotation().toRotation2d().plus(Rotation2d.k180deg)));

        double deltaH = tagPose.getZ() - robotToCamera.getZ();
        double pitch = Units.degreesToRadians(target.getPitch());
        double yaw = Units.degreesToRadians(target.getYaw());

        Transform3d unitVecUnrotated = rotate(new Rotation3d(0, -pitch, 0)).plus(rotate(new Rotation3d(0, 0, -yaw))).plus(new Transform3d(1, 0, 0, new Rotation3d()));
        Transform3d unitVec = new Transform3d(unitVecUnrotated.getTranslation(), new Rotation3d());
        Transform3d tagToRobot = unitVec.times(- deltaH / unitVec.getZ()).plus(cameraToRobot);

        Rotation3d gyroError = new Rotation3d(robotPose.getRotation()).minus(tagPose.plus(tagToRobot).getRotation());
        Transform3d tagToRobotRotated = rotate(gyroError).plus(tagToRobot);
        Pose3d visionRobotPose = tagPose.plus(tagToRobotRotated);

        Logger.recordOutput("Visionthingy/pose", visionRobotPose.toPose2d());
        Logger.recordOutput("Visionthingy/unrotated", tagPose.plus(tagToRobot).toPose2d());
        Logger.recordOutput("Visionthingy/pitch", pitch);
        Logger.recordOutput("Visionthingy/yaw", yaw);

        // Add observations
        // poseObservations.add(
        //     new PoseObservation(
        //         result.getTimestampSeconds(), // Timestamp
        //         visionRobotPose, // 3D pose estimate
        //         0.0, // Ambiguity
        //         1, // Tag count
        //         tagToRobot.getTranslation().getNorm(), // Average tag distance
        //         PoseObservationType.PHOTONVISION)); // Observation type

        targetObservations.add(
            new TargetObservation(
                timestamp,
                target.getYaw(),
                target.getPitch(),
                0.0, tagId, 0.0)
        );

        tagIds.add((short) tagId);
      } else {
        targetObservations.add(
          new TargetObservation(0.0, 0.0, 0.0,0.0, 0, 0.0)
        );
      }

      // Save target observations to inputs object
      inputs.latestTargetObservations = new TargetObservation[targetObservations.size()];
      for (int i = 0; i < targetObservations.size(); i++) {
        inputs.latestTargetObservations[i] = targetObservations.get(i);
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
