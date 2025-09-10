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
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;

public class PIDDriveCmd extends Command{
    private final Pose2d goalPos;

    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    private Rotation2d direction;

    private final double POSITION_TOLERANCE_METERS = 0.02;
    private final double VELOCITY_TOLERANCE_MPS = 0.1;
    private final double ANGLE_TOLERANCE_DEGREES = 2.0;

    public PIDDriveCmd(Pose2d goal){
        addRequirements(CatzDrivetrain.Instance);

        this.goalPos = goal;

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
        this.rotationController = new ProfiledPIDController(0.1, 0.0, 0.0, rotationConstraints);

        this.rotationController.enableContinuousInput(0, 360);
    }

    @Override
    public void initialize(){
        Logger.recordOutput("PID Target Pose", goalPos);
    }

    @Override
    public void execute(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();

        Translation2d poseError = goalPos.minus(currentPose).getTranslation();
        double currentDistance = poseError.getNorm();
        direction = poseError.getAngle();
        Logger.recordOutput("Pose Error Angle", direction.getDegrees());

        // The goal of the translation controller is to drive the distance error to zero
        double targetVel = Math.abs(translationController.calculate(currentDistance, 0.0));
        Logger.recordOutput("Pose Error Target Vel", targetVel);
        Logger.recordOutput("Pose Error Cosine", direction.getCos());
        Logger.recordOutput("Pose Error Sine", direction.getSin());

        // The goal of the rotation controller is to drive the angle to the target angle
        Logger.recordOutput("Pose Error RotDif", MathUtil.inputModulus(MathUtil.inputModulus(goalPos.getRotation().getDegrees(), 0.0, 360.0) - currentPose.getRotation().getDegrees(), 0.0, 360.0));
        double targetOmega = rotationController.calculate(MathUtil.inputModulus(currentPose.getRotation().getDegrees(), 0.0, 360.0), MathUtil.inputModulus(goalPos.getRotation().getDegrees(), 0.0, 360.0));
        Logger.recordOutput("Pose Error Omega", targetOmega);

        ChassisSpeeds goalChassisSpeeds = new ChassisSpeeds(targetVel * direction.getCos(), targetVel * direction.getSin(), targetOmega);
        CatzDrivetrain.Instance.drive(goalChassisSpeeds);

        CatzDrivetrain.Instance.setDistanceError(currentDistance);
    }

    @Override
    public boolean isFinished(){
        return isAtTargetState();
    }

    private boolean isAtTargetState(){
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        ChassisSpeeds currentSpeed = CatzRobotTracker.Instance.getRobotChassisSpeeds();

        double distanceError = currentPose.getTranslation().getDistance(goalPos.getTranslation());

        double linearVelocity = Math.hypot(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);

        double rotationError = Math.abs(goalPos.minus(currentPose).getRotation().getDegrees());

        return distanceError < POSITION_TOLERANCE_METERS && linearVelocity < VELOCITY_TOLERANCE_MPS && rotationError < ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finished!!!!!! yayayay");

        CatzDrivetrain.Instance.drive(new ChassisSpeeds());
    }
}
