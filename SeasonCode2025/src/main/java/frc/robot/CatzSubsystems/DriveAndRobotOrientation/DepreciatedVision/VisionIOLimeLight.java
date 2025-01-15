package frc.robot.CatzSubsystems.DriveAndRobotOrientation.DepreciatedVision;

import org.littletonrobotics.junction.Logger;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.DepreciatedVision.VisionConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Utilities.LimelightHelpers.LimelightResults;
import frc.robot.Utilities.LimelightHelpers.PoseEstimate;
import frc.robot.Utilities.LimelightHelpers.RawDetection;
import frc.robot.Utilities.LimelightHelpers.RawFiducial;

public class VisionIOLimeLight implements VisionIO {
    
    public String name;
    public boolean getTarget;

    private Pose2d prevVisionPos = null;
    private Pose2d visionPose2d = null;
    private boolean usableData = false;

    private Supplier<Double> gyroYaw =  () -> CatzRobotTracker.getInstance().getOdometryPose().getRotation().getDegrees();  

     /**
     * Implements Limelight camera
     *
     * @param name Name of the limelight used, and should be configured in limelight software first
     */
    public VisionIOLimeLight(String name, Transform3d transform3d) {
        LimelightHelpers.setPipelineIndex(name, LIMELIGHT_PIPLINE_APRILTAG);

        LimelightHelpers.setCameraPose_RobotSpace(name, transform3d.getX(),
                                                        transform3d.getY(),
                                                        transform3d.getZ(),
                                                        transform3d.getRotation().getX(),
                                                        transform3d.getRotation().getY(),
                                                        transform3d.getRotation().getZ()

        );
        LimelightHelpers.setStreamMode_Standard(name);

        LimelightHelpers.setLEDMode_ForceBlink(name);
        this.name = name;
        System.out.println("Limeilight " + name + " instantiated" + LimelightHelpers.getCurrentPipelineType(name));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        //------------------------------------------------------------------------------------------------------
        //  Limelight Data Collection
        //------------------------------------------------------------------------------------------------------        // Get raw AprilTag/Fiducial data
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        for (RawFiducial fiducial : fiducials) {
            inputs.id = fiducial.id;                    // Tag ID
            inputs.txnc = fiducial.txnc;             // X offset (no crosshair)
            inputs.tync = fiducial.tync;             // Y offset (no crosshair)
            inputs.ta = fiducial.ta;                 // Target area
            inputs.distToCamera = fiducial.distToCamera;  // Distance to camera
            inputs.distToRobot = fiducial.distToRobot;    // Distance to robot
            inputs.ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
        }


        //load up raw apriltag values for distance calculations
        inputs.hasTarget = LimelightHelpers.getTV(name); //whether the limelight has any vaild targets
        inputs.ty = LimelightHelpers.getTY(name); //vertical offset from crosshair to target in degrees
        inputs.tx = LimelightHelpers.getTX(name); //horizontal offset from crosshair to target
        inputs.ta = LimelightHelpers.getTA(name); //target area of the limelight from 0%-100%...how much does the apirltage take up on the frame
        inputs.primaryApriltagID = LimelightHelpers.getFiducialID(name);

        // calculates total latency using 7th table item in array 
        inputs.totalLatency = (LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name)) / 1000; //data[6] or latency is recorded in ms; divide by 1000 to get s     
        //------------------------------------------------------------------------------------------------------
        //  Null Filtering Limelight poses
        //------------------------------------------------------------------------------------------------------
        if (inputs.hasTarget) {
            // sets input timestamp
            inputs.isNewVisionPose = true;
            
            //take new pose info from the limelight api
            // collects pose information based off network tables and orients itself depending on alliance side
            LimelightHelpers.SetRobotOrientation(name, gyroYaw.get(), 0.0, 0.0, 0.0, 0.0, 0.0);
            PoseEstimate visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            inputs.isMegaTag2 = visionEstimate.isMegaTag2;
            inputs.tagCount = visionEstimate.tagCount;
            visionPose2d = visionEstimate.pose;

            //set a previous vision position depending on if we see an apriltag
            if(prevVisionPos == null){
                prevVisionPos = visionPose2d;
            }

            //logic for filtering bad estimates
            double error = visionPose2d.getTranslation().getDistance(prevVisionPos.getTranslation());
            
            //------------------------------------------------------------------------------------------------
            // error should be greater than 0 but less than 0.1 for vision estimates to be considered usable
            //----------------------------------------------------------------------------------------------
            if(error == 0.0) { //TODO why is this here?
                //obtained a null error reading
                inputs.isNewVisionPose = false;
                return;
            }
            if(error > 0.1){
                usableData = false;
            }

            if(usableData){
                //set new vision data if reading is considered good
                inputs.x = visionPose2d.getX();
                inputs.y = visionPose2d.getY();
                inputs.rotation = visionPose2d.getRotation().getRadians();
                inputs.timestamp = Timer.getFPGATimestamp() - inputs.totalLatency;
            }
            
            prevVisionPos = visionPose2d;
            usableData = false;           
        } else {
            
            //limelight is not correct and doesn't currently have a target to look for
            inputs.isNewVisionPose = false;
            prevVisionPos = null;
        }
    } //end of updateInputs(VisionIOInputs inputs)

    @Override
    public String getName() {
        return name;
    }
}