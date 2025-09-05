package frc.robot.Commands.DriveAndRobotOrientationCmds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;

public class PIDDriveCmd extends Command{
    private final Pose2d goalPos;

    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private Rotation2d direction;

    // Define tolerances for position and velocity to determine if the goal is reached
    private final double POSITION_TOLERANCE_METERS = 0.05; // 5 centimeters
    private final double VELOCITY_TOLERANCE_MPS = 0.1;     // 10 centimeters per second

    public PIDDriveCmd(Pose2d goal){
        addRequirements(CatzDrivetrain.Instance);

        this.goalPos = goal;

        // Note: These PID and TrapezoidProfile values are starting points.
        // They will need to be tuned for your specific robot's characteristics.

        // Configure the translation controller
        var translationConstraints = new TrapezoidProfile.Constraints(
            3.0, // Max velocity in meters per second
            2.0  // Max acceleration in meters per second squared
        );
        this.translationController = new ProfiledPIDController(2.5, 0.0, 0.0, translationConstraints);

        // Configure the rotation controller
        var rotationConstraints = new TrapezoidProfile.Constraints(
            360.0, // Max angular velocity in degrees per second
            720.0  // Max angular acceleration in degrees per second squared
        );
        this.rotationController = new ProfiledPIDController(0.4, 0.0, 0.0, rotationConstraints);

        // Enable continuous input for the rotation controller to handle wrapping from -180 to 180 degrees
        this.rotationController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void initialize(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        Translation2d poseError = goalPos.minus(currentPose).getTranslation();

        direction = poseError.getAngle();
    }

    @Override
    public void execute(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();

        double currentDistance = goalPos.getTranslation().getDistance(currentPose.getTranslation());

        // The goal of the translation controller is to drive the distance error to zero
        double targetVel = translationController.calculate(currentDistance, 0.0);

        // The goal of the rotation controller is to drive the angle to the target angle
        double targetOmega = rotationController.calculate(currentPose.getRotation().getDegrees(), goalPos.getRotation().getDegrees());

        ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds(targetVel * direction.getCos(), targetVel * direction.getSin(), targetOmega);
        CatzDrivetrain.Instance.drive(goalChassisSpeeds);
    }

    @Override
    public boolean isFinished(){
        return isAtTargetState();
    }

    /**
    * Checks if the robot has reached its target position and its velocity is near zero.
    * @return true if the robot is at the target state, false otherwise.
    */
    private boolean isAtTargetState(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds currentSpeed = CatzRobotTracker.Instance.getRobotChassisSpeeds();

        // Calculate the error in distance to the target position
        double distanceError = currentPose.getTranslation().getDistance(goalPos.getTranslation());
        
        // Calculate the robot's current linear velocity magnitude
        double linearVelocity = Math.hypot(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);

        // The command is finished if the robot is within the position tolerance AND its velocity is below the tolerance
        return distanceError < POSITION_TOLERANCE_METERS && linearVelocity < VELOCITY_TOLERANCE_MPS;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain by sending zero speeds
        CatzDrivetrain.Instance.drive(new ChassisSpeeds());
    }
}
