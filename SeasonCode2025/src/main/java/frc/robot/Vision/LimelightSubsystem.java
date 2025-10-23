package frc.robot.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Vision.LimelightConstants.LimelightConfig;

public class LimelightSubsystem extends SubsystemBase {

	public static LimelightSubsystem Instance = new LimelightSubsystem();

	private final VisionIO[] ios;

	private LimelightSubsystem() {
		ios = LimelightConstants.LIMELIGHT_ARRAY;

		if (Robot.isReal()) {
			for(VisionIOLimelight limelight : LimelightConstants.LIMELIGHT_ARRAY){
				LimelightConfig config = limelight.getConfig();

				LimelightHelpers.setCameraPose_RobotSpace(
						config.name,
						config.robotToCameraOffset.getX(),
						config.robotToCameraOffset.getY(),
						config.robotToCameraOffset.getZ(),
						Units.radiansToDegrees(
								config.robotToCameraOffset.getRotation().getX()),
						Units.radiansToDegrees(
								config.robotToCameraOffset.getRotation().getY()),
						Units.radiansToDegrees(
								config.robotToCameraOffset.getRotation().getZ()));
			}
		}
	}

	@Override
	public void periodic() {
		for(int i = 0; i < ios.length; i++){
			LimelightConstants.LIMELIGHT_ARRAY[i].update();
			ios[i].update();
		}
	}

	public boolean isSeeingApriltag(){
		for(VisionIO io : ios){
			return io.getNumTags() > 0;
		}
		System.out.println("Missing VisionIOs!!!!!!");
		return false;
	}
}
