// VisionIOLimelight.java
package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.LimelightHelpers.LimelightResults;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.LimelightHelpers.LimelightTarget_Fiducial;


import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;


/** IO implementation for real Limelight hardware using LimelightHelpers. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier = () -> CatzRobotTracker.Instance.getEstimatedPose().getRotation();
  
  public final String name;

  public VisionIOLimelight(String name) {
    this.name = name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    inputs.connected = results.valid;

    LimelightHelpers.SetRobotOrientation(name, rotationSupplier.get().getDegrees(), 0.0, 0.0, 0, 0, 0);

    if (!inputs.connected) {
      inputs.latestTargetObservations = new TargetObservation[0];
      inputs.poseObservations = new PoseObservation[0];
      inputs.tagIds = new int[0];
      inputs.ta = 0.0;
      return;
    }

    inputs.ta = results.botpose_avgarea;
    
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    List<TargetObservation> targetObservations = new LinkedList<>();
    
    LimelightTarget_Fiducial[] fiducials = results.targets_Fiducials;

    // --- Pre-calculate aggregate data from all visible fiducials ---
    int tagCount = fiducials.length;
    double totalDistance = 0.0;
    double maxAmbiguity = 0.0;
    
    for (LimelightTarget_Fiducial target : fiducials) {
      tagIds.add((int) target.fiducialID);
      Translation3d cameraSpaceTranslation = target.getTargetPose_CameraSpace().getTranslation();
      totalDistance += cameraSpaceTranslation.getNorm();
      maxAmbiguity = Math.max(maxAmbiguity, 0.0);
      
      targetObservations.add(
          new TargetObservation(
            results.timestamp_RIOFPGA_capture,
            target.tx,
            target.ty,
            0.0,
            (int) target.fiducialID,
            cameraSpaceTranslation.getNorm())
      );
    }
    double avgTagDistance = (tagCount > 0) ? totalDistance / tagCount : 0;


    //----------------------------------------------------------------------------------------------
    // Megatag 1 estimation
    //----------------------------------------------------------------------------------------------
    Pose3d botpose_mega1 = LimelightHelpers.getBotPose3d_wpiBlue(name);
    if (!botpose_mega1.equals(new Pose3d())) { // Check if the pose is valid (not zeroed out)
        poseObservations.add(
            new PoseObservation(
                results.timestamp_RIOFPGA_capture,
                botpose_mega1, // Use the Pose3d object directly
                maxAmbiguity,  // Use calculated max ambiguity
                tagCount,      // Use calculated tag count
                avgTagDistance,// Use calculated average distance
                PoseObservationType.MEGATAG_1)
        );
    }
    
    //----------------------------------------------------------------------------------------------
    // Megatag 2 estimation
    //----------------------------------------------------------------------------------------------
    Pose3d botpose_mega2 = LimelightHelpers.getBotPose3d_wpiBlue(name);
    if (!botpose_mega2.equals(new Pose3d())) { // Check if the pose is valid
        poseObservations.add(
            new PoseObservation(
                results.timestamp_RIOFPGA_capture,
                botpose_mega2, // Use the Pose3d object directly
                0.0,           // Ambiguity is zeroed, as MegaTag2 is fully disambiguated
                tagCount,      // Use calculated tag count
                avgTagDistance,// Use calculated average distance
                PoseObservationType.MEGATAG_2)
        );
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.latestTargetObservations = targetObservations.toArray(new TargetObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.name = name;
  }

  // The parsePose method is no longer needed with the new LimelightHelpers API.
  // private static Pose3d parsePose(double[] rawLLArray) { ... }
}