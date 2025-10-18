package frc.robot.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
public class LimelightConstants {

	public static final int kEnabledPipeline = 0;
	public static final int kDisabledPipeline = 1; //TODO try fiddling with pipelines through website
	public static final Vector<N3> enabledVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);


	public static final VisionIOLimelight[] LIMELIGHT_ARRAY = new VisionIOLimelight[] {
		new VisionIOLimelight(new LimelightConfig("limelight-udon", new Pose3d())), //TODO fill
		new VisionIOLimelight(new LimelightConfig("limelight-ramen", new Pose3d()))
	};

	//TODO Use this instead of vision shift for auto aim
	public static final int agreedTranslationUpdatesThreshold = 100;
	public static final Distance agreedTranslationUpdateEpsilon = Units.Centimeters.of(10.0);

	public static class LimelightConfig {
		public String name = "no-name-assigned";
		public Pose3d robotToCameraOffset = new Pose3d(); 
		public Vector<N3> aprilTagVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);

		public LimelightConfig(String name, Pose3d robotToCameraOffset){
			this.name = name;
			this.robotToCameraOffset = robotToCameraOffset;
		}

		public LimelightConfig(){}
	}
}
