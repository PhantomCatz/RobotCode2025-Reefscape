package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class LimelightOffsetField {
    public static AprilTagFieldLayout createLayout() {
        List<AprilTag> customTags = new ArrayList<>();

        customTags.add(new AprilTag(12, new Pose3d(new Translation3d(0.0, -0.29845, 0.1397), new Rotation3d())));
        customTags.add(new AprilTag(18, new Pose3d(new Translation3d(0.0, 0.29845, 0.47244), new Rotation3d())));

        double fieldLength = 8.0; // meters
        double fieldWidth = 4.0; // meters

        return new AprilTagFieldLayout(customTags, fieldLength, fieldWidth);
    }

    public static void exportCustomField(AprilTagFieldLayout field) {
        try {
            field.serialize(Paths.get("custom_apriltag_field.json"));
            System.out.println("Custom AprilTag field layout saved to custom_apriltag_field.json");
        } catch (IOException e) {
            System.err.println("Error saving layout: " + e.getMessage());
        }
    }
}
