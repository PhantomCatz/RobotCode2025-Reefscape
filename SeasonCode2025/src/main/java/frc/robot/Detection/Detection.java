package frc.robot.Detection;


public class Detection extends DetectionSubsystem<DetectionIOLimelight> {
	public static final Detection mInstance = new Detection();

	private Detection() {
		super(DetectionConstants.getDetectionIOConfig(), DetectionConstants.getDetectionIO());
	}

	@Override
	public void periodic() {
		super.periodic();
		// LogUtil.recordPose3d(
				// "Detection/ Camera Pose Robot Space",
				// LimelightHelpers.getCameraPose3d_RobotSpace(DetectionConstants.kLimelightName));
	}
}
