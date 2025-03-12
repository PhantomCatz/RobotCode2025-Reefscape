// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;


import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier = () -> CatzRobotTracker.getInstance().getEstimatedPose().getRotation();
  private final DoubleArrayPublisher orientationPublisher;

  // private final SwerveDrivePoseEstimator

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber taSubscriber;

  private final DoubleSubscriber tySubscriber;
  private final DoubleSubscriber tagIDSubscriber;
  private final DoubleArraySubscriber cameraSpaceSubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name) {
    var table            = NetworkTableInstance.getDefault().getTable(name);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber    = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber         = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber         = table.getDoubleTopic("ty").subscribe(0.0);
    taSubscriber         = table.getDoubleTopic("ta").subscribe(0.0);

    tagIDSubscriber      = table.getDoubleTopic("tid").subscribe(0.0);
    cameraSpaceSubscriber= table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    megatag1Subscriber   = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber   = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double usedTimestamp = Timer.getFPGATimestamp();
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = (usedTimestamp - latencySubscriber.getLastChange()) < 250;

    // Update orientation for MegaTag 2
    orientationPublisher.accept(new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0}); //TODO possible sync error

    inputs.ta = taSubscriber.get();
    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    List<TargetObservation> targetObservations = new LinkedList<>();

    //----------------------------------------------------------------------------------------------
    // Single tag Pose estimation
    //----------------------------------------------------------------------------------------------
    for (var rawSample : cameraSpaceSubscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      Translation3d cameraSpaceTranslation = new Translation3d(
        rawSample.value[0],
        rawSample.value[1],
        rawSample.value[2]
        );
        targetObservations.add(
          new TargetObservation(
            usedTimestamp,
            rawSample.value[0],
            rawSample.value[1],
            rawSample.value[2],
            (int) tagIDSubscriber.get(),
            cameraSpaceTranslation.getNorm())
            );
          }

          //----------------------------------------------------------------------------------------------
          // Megatag 1 estimation
          //----------------------------------------------------------------------------------------------
          // for (var rawSample : megatag1Subscriber.readQueue()) {
          //   // if sample is invalid, skip
          //   if (rawSample.value.length == 0) continue;
          //   poseObservations.add(
          //     new PoseObservation(
          //       // Timestamp, based on server timestamp of publish and latency
          //       usedTimestamp - rawSample.value[6] * 1.0e-3,

          //       // 3D pose estimate
          //       parsePose(rawSample.value),

          //       // Ambiguity, zeroed because the pose is already disambiguated
          //     rawSample.value[8],

          //     // Tag count
          //     (int) rawSample.value[7],

          //     // Average tag distance
          //     rawSample.value[9], // Used to grab area at value 10 //TODO

          //     // Observation type
          //     PoseObservationType.MEGATAG_1)
          //     );
          //   }

            NetworkTableInstance.getDefault().flush(); // Increases network traffic but recommended by Limelight

    // System.out.println("lastchange: " + megatag2Subscriber.getLastChange() * 1e-6);
    // System.out.println("cur: " + usedTimestamp);
    //----------------------------------------------------------------------------------------------
    // Megatag 2 estimation
    //----------------------------------------------------------------------------------------------
    for (var rawSample : megatag2Subscriber.readQueue()) {
      // System.out.println("latency: " + rawSample.value[6]);
      // if sample is invalid, skip
      if (rawSample.value.length == 0) continue;
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              usedTimestamp - ((rawSample.value[6] + VisionConstants.EXTRA_LATENCY_MS) * 1.0e-3),

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9], // Used to grab area at value 10 //TODO

              // Observation type
              PoseObservationType.MEGATAG_2)
      );
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save Tx Ty observation to inputs object
    inputs.latestTargetObservations = new TargetObservation[targetObservations.size()];
    for (int i = 0; i < targetObservations.size(); i++) {
      inputs.latestTargetObservations[i] = targetObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
