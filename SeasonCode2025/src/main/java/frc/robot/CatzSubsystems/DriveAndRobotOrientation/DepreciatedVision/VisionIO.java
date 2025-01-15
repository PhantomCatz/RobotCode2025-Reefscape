package frc.robot.CatzSubsystems.DriveAndRobotOrientation.DepreciatedVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    
    @AutoLog
    public class VisionIOInputs {
        public double x; //new x pose estimator coordinate value based off limelight
        public double y; //new y pose estimator coordinate value based off limelight
        public double rotation; //new rotation pose estimator coordinate value based off limelight
        public double timestamp;
        public boolean isNewVisionPose; // is it a new pose from a new apriltag estimate

        public int id;                 // Tag ID
        public double txnc;         // X offset (no crosshair)
        public double tync;         // Y offset (no crosshair)
        public double ta_Fiducial;   // Target area
        public double distToCamera;  // Distance to camera
        public double distToRobot;    // Distance to robot
        public double ambiguity;   // Tag pose ambiguity

        public double primaryApriltagID; //closest apirltag id that the limelight is communicating with

        public double ty; //vertical offset from crosshair to target in degrees
        public double tx; //horizontal offset from crosshair to target
        public boolean hasTarget; //whether the limelight has any vaild targets
        public double ta; //target area of the limelight from 0%-100%...how much does the apirltage take up on the frame

        public int tagCount;
        public double totalLatency;

        public boolean isMegaTag2;


        // Nerual detector inputs
        public int classID;
        public double corner0X;
        public double corner0Y;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default String getName() {
        return "";
    }

    public default void setReferencePose(Pose2d pose) {}

    public default void getHorizontalAngle() {}
    
}
