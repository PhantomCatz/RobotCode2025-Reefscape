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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
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

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double goalHeightInches = 8.75;

        double angleToGoalDegrees = result.getBestTarget().getPitch();
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate distance
        double distanceToTargetMeters = Math.abs((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians));

        targetObservations.add(
            new TargetObservation(
                timestamp,
                result.getBestTarget().getYaw(),
                result.getBestTarget().getPitch(),
                0.0, result.getBestTarget().fiducialId, distanceToTargetMeters)
        );
      } else {
        targetObservations.add(
          new TargetObservation(0.0, 0.0, 0.0,0.0, 0, 0.0)
        );
      }
      inputs.latestTargetObservations = new TargetObservation[targetObservations.size()];
      for (int i = 0; i < targetObservations.size(); i++) {
        inputs.latestTargetObservations[i] = targetObservations.get(i);
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distace
                PoseObservationType.PHOTONVISION)); // Observation type
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
