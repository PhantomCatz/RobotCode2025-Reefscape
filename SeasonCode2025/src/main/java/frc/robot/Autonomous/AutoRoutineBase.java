package frc.robot.Autonomous;

import java.util.Set;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;

public class AutoRoutineBase {
    private AutoRoutine routine;

    public AutoRoutineBase(String name){
        routine = CatzConstants.autoFactory.newRoutine(name);
    }

    protected void prepRoutine(Command... sequence){
        routine.active().onTrue(
            Commands.sequence(sequence).alongWith(new PrintCommand("asdfasdfasdf"))
        );
    }

    protected Command followTrajectoryAndScore(AutoTrajectory trajectory, int level){
        return Commands.parallel
        (
            Commands.sequence
            (
                Commands.waitUntil(CatzDrivetrain.Instance::closeEnoughToRaiseElevator),
                CatzSuperstructure.Instance.LXElevator(level)
            ),
            Commands.sequence
            (
                followTrajectoryWithAccuracy(trajectory),
                Commands.waitUntil(CatzElevator.Instance::isElevatorInPos),
                CatzSuperstructure.Instance.LXCoral(level)
            )
        );
    }

    protected Command followTrajectoryAndIntake(AutoTrajectory traj){
        return Commands.parallel
        (
            Commands.sequence
            (
                Commands.waitUntil(CatzDrivetrain.Instance::closeEnoughToStartIntake),
                CatzSuperstructure.Instance.intakeCoralStation()
            ),
            traj.cmd()
        );
    }

    protected Command waitUntilCoralIntaked(){
        return Commands.waitUntil(() -> CatzOuttake.Instance.isDesiredCoralState(false));
    }

    protected Command followTrajectoryWithAccuracy(AutoTrajectory traj){
        return Commands.defer(() ->
                                new FunctionalCommand
                                (
                                    () -> {CatzDrivetrain.Instance.followChoreoTrajectoryInit(); traj.cmd().initialize();},
                                    traj.cmd()::execute,
                                    traj.cmd()::end,
                                    () -> isAtPose(traj)
                                ),
                                Set.of(CatzDrivetrain.Instance)
                             );
    }

    private boolean isAtPose(AutoTrajectory trajectory){
        boolean isAtTrans = translationIsFinished(trajectory, AutonConstants.ACCEPTABLE_DIST_METERS);
        boolean isAtRot = rotationIsFinished(trajectory, AutonConstants.ACCEPTABLE_ANGLE_DEG);
        System.out.println("finished? " + isAtRot + isAtTrans);

        return isAtTrans && isAtRot;
    }

    private boolean rotationIsFinished(AutoTrajectory trajectory, double epsilonAngleDeg){
        Rotation2d curRot = CatzRobotTracker.Instance.getEstimatedPose().getRotation();
        Rotation2d goalRot = trajectory.getFinalPose().get().getRotation();

        return Math.abs(goalRot.minus(curRot).getDegrees()) % 360 < epsilonAngleDeg;
    }

    private boolean translationIsFinished(AutoTrajectory trajectory, double epsilonDist) {
		Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
		Pose2d finalPose = trajectory.getFinalPose().get();


		return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist;
	}

    protected AutoTrajectory getTrajectory(String name){
        return routine.trajectory(name);
    }

    protected AutoTrajectory getTrajectory(String name, int index){
        return routine.trajectory(name, index);
    }

    public AutoRoutine getRoutine(){
        return routine;
    }
}
