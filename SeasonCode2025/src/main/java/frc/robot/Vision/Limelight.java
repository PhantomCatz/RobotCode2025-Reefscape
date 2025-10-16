package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.Vision.LimelightSubsystem;
import frc.robot.Vision.VisionIOLimelight;

public class Limelight extends LimelightSubsystem<VisionIOLimelight> {
	public static final Limelight Instance = new Limelight();

	private Pose2d lastPose = new Pose2d();
	private long numPoseStableUpdates = 0;

	private Limelight() {
		super(LimelightConstants.getVisionIOConfig(), LimelightConstants.getVisionIO());
	}

	@Override
	public void periodic() {
		try {
			super.periodic();
			Pose2d ioPose = io.getLatestEstimate();
			if (ioPose != lastPose) {
				if (CatzRobotTracker.Instance.getEstimatedPose().getTranslation().getDistance(ioPose.getTranslation())
						< LimelightConstants.agreedTranslationUpdateEpsilon.in(Units.Meters)) {
					numPoseStableUpdates++;
				} else {
					numPoseStableUpdates = 0;
				}
			}

			lastPose = ioPose;

			SmartDashboard.putNumber("Vision/Num Agreed Stable Updates", numPoseStableUpdates);
		} catch (Exception e) {
			SmartDashboard.putNumber("Limelight/Crash", Timer.getFPGATimestamp());
			SmartDashboard.putString("Limelight/Crash Exception", e.getMessage());
			SmartDashboard.putString(
					"Limelight/Crash Stacktrace", e.getStackTrace().toString());
		}
	}

	public Time getLastUpdateTime() {
		return io.getLatestEstimateTime();
	}

    public boolean isSeeingApriltag() {
        return (io.getLatestEstimateNumTags() > 0);
    }

	public Pose2d getLatestUpdate() {
		return lastPose;
	}

	public boolean getPoseStable() {
		return numPoseStableUpdates > LimelightConstants.agreedTranslationUpdatesThreshold;
	}

}