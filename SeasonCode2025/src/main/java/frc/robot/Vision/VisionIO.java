package frc.robot.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Utilities.LimelightHelpers.PoseEstimate;

public interface VisionIO {

	public default void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {};

	public default void update() {};

	public default void updateInputs(VisionIOInputs inputs) {}

	int getNumTags();

	@AutoLog
	public static class VisionIOInputs {
		public boolean hasTargets = false;
		public Pose2d calculatedPose = new Pose2d();
	}
}
