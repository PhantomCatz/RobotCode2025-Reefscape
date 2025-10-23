package frc.robot.Vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Utilities.LimelightHelpers.PoseEstimate;
import frc.robot.Vision.LimelightConstants.LimelightConfig;

public class VisionIOLimelight implements VisionIO {
	private Pose2d latestEstimate = new Pose2d();
	private Time latestEstimateTime = Units.Seconds.of(0.0);
    private int latestEstimateNumTags = 0;
	private LimelightConfig config = new LimelightConfig();


	public VisionIOLimelight(LimelightConfig config){
		this.config = config;
	}

	@Override
	public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {
		if(poseEstimate == null) return;
		SmartDashboard.putNumber(config.name + "/Tag Count", poseEstimate.tagCount);
		SmartDashboard.putNumber(config.name + "/FGPA Timestamp", Timer.getFPGATimestamp());
		SmartDashboard.putNumber(
				config.name + "/Estimate to FGPA Timestamp", Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds));
        latestEstimateNumTags = poseEstimate.tagCount;
		if (poseEstimate.tagCount >= minTagNum) {
			latestEstimate = poseEstimate.pose;
			latestEstimateTime = Units.Seconds.of(poseEstimate.timestampSeconds);
			CatzRobotTracker.Instance.addVisionObservation(
                new VisionObservation(config.name, poseEstimate.pose, poseEstimate.timestampSeconds, LimelightConstants.enabledVisionStdDevs.times(poseEstimate.avgTagDist))
			);
		}
	}

	public Pose2d getLatestEstimate() {
		return latestEstimate;
	}

	public Time getLatestEstimateTime() {
		return latestEstimateTime;
	}

    public int getLatestEstimateNumTags() {
        return latestEstimateNumTags;
    }

	@Override
	public void update() {
		updateGyro();
		setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name), 1);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs){
		inputs.hasTargets = latestEstimateNumTags > 0;
		inputs.calculatedPose = latestEstimate;
	}

	@Override
	public int getNumTags(){
		return latestEstimateNumTags;
	}

	private void updateGyro() {
		Rotation2d theta = CatzRobotTracker.Instance.getEstimatedPose().getRotation();
		LimelightHelpers.SetRobotOrientation(config.name, theta.getDegrees(), 0, 0, 0, 0, 0);
	}

	public void updateConfig(LimelightConfig config) {
		this.config = config;
	}

	public LimelightConfig getConfig(){
		return config;
	}
}
