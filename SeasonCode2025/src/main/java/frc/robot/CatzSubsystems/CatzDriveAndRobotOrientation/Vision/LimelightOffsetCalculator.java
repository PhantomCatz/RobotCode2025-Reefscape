package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Calculates the physical offset of a Limelight camera on a robot.
 *
 * <p>This class uses a known robot position and the position reported by the Limelight
 * (which initially has zero offsets) to determine the camera's translational and rotational
 * offset from the robot's center.
 */
public class LimelightOffsetCalculator {

    /**
     * Represents the calculated offsets of the Limelight.
     *
     * <p>This simple data structure holds the translational (x, y) and rotational (theta)
     * offsets.
     */
    public static class LimelightOffsets {
        public final double offsetX;
        public final double offsetY;
        public final Rotation2d offsetRotation;

        /**
         * Constructs a LimelightOffsets object.
         *
         * @param offsetX The calculated offset in the x-direction (meters).
         * @param offsetY The calculated offset in the y-direction (meters).
         * @param offsetRotation The calculated rotational offset.
         */
        public LimelightOffsets(double offsetX, double offsetY, Rotation2d offsetRotation) {
            this.offsetX = offsetX;
            this.offsetY = offsetY;
            this.offsetRotation = offsetRotation;
        }

        @Override
        public String toString() {
            return String.format(
                "Limelight Offsets:\n" +
                "  X: %.4f meters\n" +
                "  Y: %.4f meters\n" +
                "  Rotation: %.4f degrees",
                offsetX, offsetY, offsetRotation.getDegrees()
            );
        }
    }

    /**
     * Calculates the Limelight's physical offsets based on a known robot pose.
     *
     * @param knownRobotPose The true, measured pose of the robot on the field.
     * @param limelightReportedPose The pose calculated by the Limelight, assuming zero offsets.
     * @return A {@link LimelightOffsets} object containing the calculated x, y, and rotational offsets.
     */
    public static LimelightOffsets calculateOffsets(Pose2d knownRobotPose, Pose2d limelightReportedPose) {
        // The transform from the robot's actual pose to the pose reported by the Limelight
        // represents the offset of the Limelight itself.
        // We can find this transform by getting the difference between the two poses.
        Transform2d offsetTransform = knownRobotPose.minus(limelightReportedPose);

        // The translation component of the transform is the (x, y) offset of the Limelight.
        Translation2d translationOffset = offsetTransform.getTranslation();

        // The rotation component of the transform is the rotational offset.
        Rotation2d rotationOffset = offsetTransform.getRotation();

        return new LimelightOffsets(translationOffset.getX(), translationOffset.getY(), rotationOffset);
    }
}
