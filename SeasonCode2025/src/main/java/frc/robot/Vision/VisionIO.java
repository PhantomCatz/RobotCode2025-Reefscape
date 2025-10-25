package frc.robot.Vision;

import frc.robot.Utilities.LimelightHelpers.PoseEstimate;

public abstract class VisionIO {
	protected boolean disabled = false;

	public void disable(boolean disable) {
		disabled = disable;
	}

	public abstract void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum);

	public abstract void update();

	public boolean getDisabled() {
		return disabled;
	}
}