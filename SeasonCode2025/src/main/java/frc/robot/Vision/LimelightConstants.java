package frc.robot.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
public class LimelightConstants {

	public static final int kEnabledPipeline = 0;
	public static final int kDisabledPipeline = 1; //TODO try fiddling with pipelines through website
	public static final Vector<N3> enabledVisionStdDevs = VecBuilder.fill(0.3, 0.3, 99999.0);


	//offsets are forward/back, left/right, up/down
	// roll (along robot y axis), pitch (along robot x axis), yaw (along robot z axis)


	public static final VisionIOLimelight[] LIMELIGHT_ARRAY = new VisionIOLimelight[] {
		new VisionIOLimelight(new LimelightConfig("limelight-udon", 
			new Pose3d(new Translation3d(0.23178, -0.2794, 0.23114), new Rotation3d(0.0, 15.0, -18.0))
		)),

		new VisionIOLimelight(new LimelightConfig("limelight-soba", 
			new Pose3d(new Translation3d(0.25, 0.281, 0.232), new Rotation3d(0.8, 15.0, 18.0))
		))
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
