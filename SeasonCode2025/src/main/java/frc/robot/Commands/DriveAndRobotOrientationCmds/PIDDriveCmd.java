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
    private final double goalVel;

    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private double targetDistance;
    private Rotation2d direction;

    public PIDDriveCmd(Pose2d goal, double goalVel, ProfiledPIDController translationController, ProfiledPIDController rotationController){
        addRequirements(CatzDrivetrain.Instance);

        this.goalPos = goal;
        this.goalVel = goalVel;

        this.translationController = translationController;
        this.rotationController = rotationController;
    }

    @Override
    public void initialize(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        Translation2d poseError = goalPos.minus(currentPose).getTranslation();

        targetDistance = poseError.getNorm();
        direction = poseError.getAngle();
    }

    @Override
    public void execute(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();

        double currentDistance = goalPos.minus(currentPose).getTranslation().getNorm();

        double targetVel = translationController.calculate(currentDistance, targetDistance);

        //TODO ensure rotation controller is continuous.
        double targetOmega = rotationController.calculate(currentPose.getRotation().getDegrees(), goalPos.getRotation().getDegrees());

        ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds(targetVel * direction.getCos(), targetVel * direction.getSin(), targetOmega);

    }
}
