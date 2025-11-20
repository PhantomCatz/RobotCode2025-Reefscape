package frc.robot.CatzSubsystems.CatzVision.Detection;


public class Detection extends DetectionSubsystem<DetectionIOLimelight> {
	public static final Detection Instance = new Detection();

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
