package frc.robot.CatzSubsystems.CatzVision.Detection;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker.VisionObservation;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightConstants;
import frc.robot.CatzSubsystems.CatzVision.ApriltagScanning.LimelightConstants.LimelightConfig;
import frc.robot.Utilities.FieldLayout;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Utilities.LimelightHelpers.PoseEstimate;
import frc.robot.Utilities.LimelightHelpers.RawDetection;
import frc.robot.Utilities.Stopwatch;
import frc.robot.Utilities.Util;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

public class DetectionIOLimelight extends DetectionIO {
	private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
	private int maxI = 0;
	private ArrayList<StructPublisher<Pose2d>> publishers = new ArrayList<StructPublisher<Pose2d>>();
	private ArrayList<Coral> newDet = new ArrayList<>();
	private AtomicReference<ArrayList<Coral>> tracker = new AtomicReference<>(newDet);
	private Pose2d closestCoralGroupPose = null;
	//private ArrayList<Coral> tracker = new ArrayList<Coral>();
	private Stopwatch mStopwatch = new Stopwatch();
	private int pipelineToSet = 0;
	private Stopwatch mResetStopwatch = new Stopwatch();
	private LimelightConfig config = new LimelightConfig();
	private final NetworkTable visTable = ntInstance.getTable("SmartDashboard/Detection");
	private final StructPublisher<Pose2d> closestCoralPose =
			visTable.getStructTopic("BestCoralPose", Pose2d.struct).publish();
	private final StructPublisher<Translation2d> closestCoralTranslation = visTable.getStructTopic(
					"BestCoralTranslation", Translation2d.struct)
			.publish();

	private Pose2d latestEstimate = new Pose2d();
	private Time latestEstimateTime = edu.wpi.first.units.Units.Seconds.of(0.0);
	protected StructPublisher<Pose2d> aprilTagPose = NetworkTableInstance.getDefault()
			.getTable("SmartDashboard/Detection/AprilTagPose")
			.getStructTopic("", Pose2d.struct)
			.publish();

	private static final double POSE_BUFFER_SIZE_SEC = 2.0;
	private final TimeInterpolatableBuffer<Pose2d> POSE_BUFFER =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SEC);

	class Coral {
		Pose2d coralPose;
		Translation2d coralTranslation;
		double detectionTime;

		public Coral(Pose2d coralPose, Translation2d coralTranslation, double detectionTime) {
			this.coralPose = coralPose;
			this.coralTranslation = coralTranslation;
			this.detectionTime = detectionTime;
		}
	}

	public enum DetectionMode {
		AUTO(DetectionConstants.kAutoPipeline),
		TELE(DetectionConstants.kTelePipeline),
		DISABLED(DetectionConstants.kDisabledPipeline);

		public int index;

		private DetectionMode(int index) {
			this.index = index;
		}
	}

	public void configLimelight(LimelightConfig config) {
		this.config = config;
	}

	@Override
	public void updateInputs(DetectionIOInputs inputs) {

		inputs.nearestCoral = getCoralPose();
		// System.out.println("nearest coral "+inputs.nearestCoral);
		mStopwatch.startIfNotRunning();
		if (pipelineToSet == LimelightHelpers.getCurrentPipelineIndex(config.name)) {
			if (pipelineToSet == DetectionMode.AUTO.index) {
				Translation2d base = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
				RawDetection[] all = LimelightHelpers.getRawDetections(config.name);
				double latencyMs = LimelightHelpers.getLatency_Capture(config.name) + LimelightHelpers.getLatency_Pipeline(config.name);
				Translation2d bestTranslation = null;
				Pose2d bestCoralPose = null;
				double now = Timer.getFPGATimestamp(); // Account for latency in storing timestamp
				Pose2d curPose = CatzRobotTracker.Instance.getEstimatedPose();
				POSE_BUFFER.addSample(now, curPose);
				Optional<Pose2d> poseFromCapture = POSE_BUFFER.getSample(now - latencyMs/1000.0);
				if (poseFromCapture != null && poseFromCapture.isEmpty()) {
					System.out.println("failing detection"+now);
					return;
				}
				newDet = new ArrayList<Coral>();
				// tracker.removeIf((coral) -> now - coral.detectionTime > 0.2);

				// while (tracker.size() > 0) {
				// 	tracker.remove(0);
				// }

				for (RawDetection detection : all) {
					if (detection.classId == 0) continue;
					double tx = detection.txnc;
					double ty = detection.tync;
					Translation2d coralTranslation = calcDistToCoral(tx, ty)
					// Logger.recordOutput("Detection/coralTranslation", coralTranslation);
							.plus(config.robotToCameraOffset.getTranslation().toTranslation2d());
					Rotation2d coralRotation = coralTranslation.getAngle().plus(Rotation2d.k180deg);
					// System.out.println(coralRotation);
					Pose2d coralPose =
						poseFromCapture.get().transformBy(new Transform2d(coralTranslation, coralRotation));


					if (FieldLayout.outsideField(coralPose)) {
						SmartDashboard.putBoolean("Outside Field", FieldLayout.outsideField(coralPose));
						// LogUtil.recordPose2d(config.name + "Last Coral Pose Outside Field", coralPose);
						continue;
					}
					newDet.add(new Coral(coralPose, coralTranslation, now - (latencyMs / 1000)));
				}

				for (Coral coral : newDet) {
					if (bestTranslation == null
							|| bestCoralPose.getTranslation().getDistance(base)
									> coral.coralPose.getTranslation().getDistance(base)) {
						bestTranslation = coral.coralTranslation;
						bestCoralPose = coral.coralPose;
					}
				}

				if (bestCoralPose != null) {
					closestCoralPose.set(bestCoralPose);
					closestCoralTranslation.set(bestTranslation);
				}
			} else if (pipelineToSet == DetectionMode.TELE.index) {
				updateAprilTagDetection();
			}
		} else if (mStopwatch.getTime().gte(Seconds.of(0.5))) {
			LimelightHelpers.setPipelineIndex(config.name, (int) LimelightHelpers.getCurrentPipelineIndex(config.name));
			mResetStopwatch.resetAndStart();
			mStopwatch.reset();
		} else {
			if (mResetStopwatch.getTime().gte(edu.wpi.first.units.Units.Seconds.of(0.5))) {
				LimelightHelpers.setPipelineIndex(config.name, pipelineToSet);
				mResetStopwatch.reset();
				mStopwatch.resetAndStart();
			}
		}
		tracker.set(newDet);
	}

	@Override
	public Pose2d getCoralPose() {
		Translation2d bestTranslation = null;
		Pose2d bestCoralPose = null;
		Translation2d robotPose = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
		for (Coral coral : tracker.get()) {
			if (bestTranslation == null
					|| bestCoralPose.getTranslation().getDistance(robotPose)
							> coral.coralPose.getTranslation().getDistance(robotPose)) {
				bestTranslation = coral.coralTranslation;
				bestCoralPose = coral.coralPose;
			}
		}
		return bestCoralPose; // will return null if no coral
	}

	private double getSquaredDistance(Translation2d iTranslation, Translation2d jTranslation) {
		double xDiff = iTranslation.getX()-jTranslation.getX();
		double yDiff = iTranslation.getY()-jTranslation.getY();
		return (xDiff*xDiff + yDiff*yDiff);
	}

	@Override
	public synchronized void setNearestGroupPose() {
		ArrayList<Coral> currentCoral = tracker.get();
		if (currentCoral.size() == 0) { // if can't see, use old pose
			return;
		}
		double now = Timer.getFPGATimestamp();
		Pose2d bestGroupCoralPose = null;
		Boolean[] visited = new Boolean[currentCoral.size()];
		Translation2d base = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
		for (int i = 0; i < currentCoral.size(); i++) {
			visited[i] = false;
		}
		// make arraylist of groups, each group hold indices of corals in the group
		ArrayList<ArrayList<Integer>> groups = new ArrayList<>();
		for (int i=0; i<currentCoral.size(); i++) {
			if (visited[i]) continue;
			visited[i] = true;
			Queue<Integer> q = new LinkedList<>();
			q.add(i);
			groups.add(new ArrayList<>());
			while (!q.isEmpty()) {
				Integer cur = q.poll();
				groups.get(groups.size()-1).add(cur);
				for (int j=0; j<currentCoral.size(); j++) {
					if (visited[j]) continue;
					if (getSquaredDistance(currentCoral.get(i).coralTranslation, currentCoral.get(j).coralTranslation) < DetectionConstants.MAX_GROUP_DIST_SQUARED) {
						visited[j] = true;
						q.add(j);
					}
				}
			}
		}
		// System.out.println("number of corals"+tracker.size());
		// System.out.println("number of groups"+groups.size());
		// for (int i=0; i<groups.size(); i++) {
		// 	System.out.println("group"+i+" size"+groups.get(i).size());
		// }
		// loop through groups and find which has best ratio
		double bestRatio = 0.0; // ratio of size of group to distance of closest coral in group
		for (int i=0; i<groups.size(); i++) {
			double closestDistInGroup = 1e9;
			Pose2d closestCoralPoseInGroup = null;
			for (int c : groups.get(i)) {
				Pose2d thisCoralPose = currentCoral.get(c).coralPose;
				double thisDist = thisCoralPose.getTranslation().getDistance(base);
				if (closestDistInGroup > thisDist) {
					closestDistInGroup = thisDist;
					closestCoralPoseInGroup = thisCoralPose;
				}
			}
			double thisRatio = groups.get(i).size() / closestDistInGroup;
			if (thisRatio > bestRatio) {
				bestRatio = thisRatio;
				bestGroupCoralPose = closestCoralPoseInGroup;
			}
		}
		double timeUsed = Timer.getFPGATimestamp() - now;
		System.out.println("group function time used: "+timeUsed);
		closestCoralGroupPose = bestGroupCoralPose;
	}

	@Override
	public synchronized Pose2d getNearestGroupPose() {
		return closestCoralGroupPose;
	}

	@Override
	public boolean txComplete(double tx) {
		return Util.epsilonEquals(tx, 0, 4);
	}

	@Override
	public int coralCount() {
		return LimelightHelpers.getTargetCount(config.name);
	}

	@Override
	public Translation2d calcDistToCoral(double tx, double ty) {
		final Distance heightFromCoral = config.robotToCameraOffset.getMeasureZ().minus(DetectionConstants.kCoralRadius);

		double totalAngleY = Units.degreesToRadians(ty) //pitch
				- config.robotToCameraOffset.getRotation().getY();
		Distance distAwayY = heightFromCoral.times(Math.tan(totalAngleY)); // robot x. forward/backward

		//if the limelight is facing backwards, you need to flip dist away because this value is calculated relative to the limelight but the actual distance needs to be relative to the robot.
		if(Math.abs(Math.toDegrees(config.robotToCameraOffset.getRotation().getZ())) > 90.0){
			distAwayY = distAwayY.times(-1); //because the LL4 facing backwards
		}


		Distance distHypotenuseYToGround = BaseUnits.DistanceUnit.of(Math.hypot( //distance from lens to coral only in the y-axis
				distAwayY.in(BaseUnits.DistanceUnit),
				heightFromCoral.in(BaseUnits.DistanceUnit)));

		double totalAngleX = Units.degreesToRadians(-tx)
				+ config.robotToCameraOffset.getRotation().getZ();

		Distance distAwayX = distHypotenuseYToGround.times(-Math.tan(totalAngleX)); // robot y. left/right

		SmartDashboard.putNumber(config.name + "/tx", tx);
		SmartDashboard.putNumber(config.name + "/ty", ty);
		Logger.recordOutput(config.name + "/Distance Away Y", distAwayY.in(edu.wpi.first.units.Units.Meters));
		Logger.recordOutput(config.name + "/Distance Away X", distAwayX.in(edu.wpi.first.units.Units.Meters));
		Logger.recordOutput(config.name + "/Total Angle Y", Units.radiansToDegrees(totalAngleY));
		Logger.recordOutput(
				config.name + "Detection/Distance Away Hyp ", distHypotenuseYToGround.in(edu.wpi.first.units.Units.Meters));

		return new Translation2d(distAwayY, distAwayX);
	}

	@Override
	public void setPipeline(int index) {
		pipelineToSet = index;
		LimelightHelpers.setPipelineIndex(config.name, index);
	}

	private void updateGyro() {
		Rotation2d theta = CatzRobotTracker.Instance.getEstimatedPose().getRotation();
		LimelightHelpers.SetRobotOrientation(config.name, theta.getDegrees(), 0, 0, 0, 0, 0);
	}

	public void updateAprilTagDetection() {
		updateGyro();
		setLatestEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name), 1);
	}

	public void setLatestEstimate(PoseEstimate poseEstimate, int minTagNum) {
		SmartDashboard.putNumber(config.name + "/Tag Count", poseEstimate.tagCount);
		SmartDashboard.putNumber(config.name + "/FGPA Timestamp", Timer.getFPGATimestamp());
		SmartDashboard.putNumber(
				config.name + "/Estimate to FGPA Timestamp", Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds));
		if (poseEstimate.tagCount >= minTagNum) {
			latestEstimate = poseEstimate.pose;
			latestEstimateTime = edu.wpi.first.units.Units.Seconds.of(poseEstimate.timestampSeconds);
			aprilTagPose.set(poseEstimate.pose);
			CatzRobotTracker.Instance.addVisionObservation(
                new VisionObservation(config.name, poseEstimate.pose, poseEstimate.timestampSeconds, LimelightConstants.enabledVisionStdDevs.times(poseEstimate.avgTagDist))
			);
		}
	}
}
