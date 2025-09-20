package frc.robot.Commands.DriveAndRobotOrientationCmds;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;

public class PIDDriveCmd extends Command{

    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private final double POSITION_TOLERANCE_METERS = 0.02;
    private final double VELOCITY_TOLERANCE_MPS = 0.1;
    private final double ANGLE_TOLERANCE_DEGREES = 2.0;
    private final double ALLOWABLE_VISION_ADJUST = 4e-3; //TODO tune

    private Pose2d goalPos;
    private boolean readyToScore = false;

    public PIDDriveCmd(Pose2d goal){
        addRequirements(CatzDrivetrain.Instance);

        CatzDrivetrain.Instance.setPIDGoalPose(goal);

        // Configure the translation controller
        var translationConstraints = new TrapezoidProfile.Constraints(
            4.0,
            4.0
        );
        this.translationController = new ProfiledPIDController(3.0, 0.0, 0.0, translationConstraints);

        // Configure the rotation controller
        var rotationConstraints = new TrapezoidProfile.Constraints(
            360.0,
            720.0
        );
        this.rotationController = new ProfiledPIDController(3.0, 0.0, 0.0, rotationConstraints);
        this.rotationController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void initialize(){
        goalPos = CatzDrivetrain.Instance.getPIDGoalPose();
        Logger.recordOutput("PID Target Pose", goalPos);
    }

    @Override
    public void execute(){
        if(readyToScore) return;

        goalPos = CatzDrivetrain.Instance.getPIDGoalPose();
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        Translation2d poseError = goalPos.minus(currentPose).getTranslation();

        double currentDistance = poseError.getNorm();
        Rotation2d direction = poseError.getAngle();
        double angleError = MathUtil.inputModulus(goalPos.getRotation().getDegrees() - currentPose.getRotation().getDegrees(), -180.0, 180.0);

        Logger.recordOutput("Pose Error Angle", angleError);

        // The goal of the translation controller is to drive the distance error to zero
        double targetVel = Math.abs(translationController.calculate(currentDistance, 0.0));
        Logger.recordOutput("Pose Error Target Vel", targetVel);
        Logger.recordOutput("Pose Error Cosine", direction.getCos());
        Logger.recordOutput("Pose Error Sine", direction.getSin());

        // The goal of the rotation controller is to drive the angle to the target angle
        double targetOmega = Math.toRadians(rotationController.calculate(0.0, angleError));
        Logger.recordOutput("Pose Error Omega", targetOmega);

        ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds(targetVel * direction.getCos(), targetVel * direction.getSin(), targetOmega);
        CatzDrivetrain.Instance.drive(goalChassisSpeeds);

        CatzDrivetrain.Instance.setDistanceError(currentDistance);
    }

    @Override
    public boolean isFinished(){
        readyToScore = isAtTargetState();// && CatzVision.Instance.isSeeingApriltag() && CatzRobotTracker.Instance.getVisionPoseShift().getNorm() < ALLOWABLE_VISION_ADJUST;
        if(readyToScore){
            RobotContainer.Instance.rumbleDrvController(0.5);
        }else{
            RobotContainer.Instance.rumbleDrvController(0.0);
        }
        return readyToScore && CatzSuperstructure.Instance.getCanShoot().get();
    }

    private boolean isAtTargetState(){
        goalPos = CatzDrivetrain.Instance.getPIDGoalPose();
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds currentSpeed = CatzRobotTracker.Instance.getRobotChassisSpeeds();

        double distanceError = currentPose.getTranslation().getDistance(goalPos.getTranslation());
        double linearVelocity = Math.hypot(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);

        double rotationError = Math.abs(MathUtil.inputModulus(goalPos.getRotation().getDegrees() - currentPose.getRotation().getDegrees(), -180.0, 180.0));
        Logger.recordOutput("Rotation Error", rotationError < ANGLE_TOLERANCE_DEGREES);
        Logger.recordOutput("Distance Error", distanceError < POSITION_TOLERANCE_METERS);
        Logger.recordOutput("Linear Velocity", linearVelocity < VELOCITY_TOLERANCE_MPS);
        return distanceError < POSITION_TOLERANCE_METERS &&
               linearVelocity < VELOCITY_TOLERANCE_MPS &&
               rotationError < ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finished!!!!!! yayayay");
        RobotContainer.Instance.rumbleDrvController(0.0);
        CatzDrivetrain.Instance.drive(new ChassisSpeeds());
    }
}
