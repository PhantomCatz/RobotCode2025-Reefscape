// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.TeleopPosSelector;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.CatzAlgaePivot;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.CatzAlgaeRemover;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Utilities.waituntil;

public class CatzStateCommands {

    public static Command driveToScore(RobotContainer robotContainer, PathPlannerPath pathToReadyPose, int level){
        final double PREDICT_DISTANCE = 0.5; //meters

        CatzDrivetrain drivetrain = robotContainer.getCatzDrivetrain();

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPose, drivetrain, true, robotContainer).deadlineFor(
                new RepeatCommand(LXElevator(robotContainer, level).alongWith(new PrintCommand("sfhoisdhfisdf")).onlyIf(() -> drivetrain.getDistanceError() < PREDICT_DISTANCE))
            ),
            LXCoral(robotContainer, level),
            stow(robotContainer)
        );
    }

    public static Command driveToCoralStation(RobotContainer robotContainer, PathPlannerPath path){
        final double PREDICT_DISTANCE = 1.0; //meters

        CatzDrivetrain drivetrain = robotContainer.getCatzDrivetrain();
        CatzOuttake outtake = robotContainer.getCatzOuttake();

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(path, drivetrain, false, robotContainer).deadlineFor(
                new RepeatCommand(intakeCoralStation(robotContainer).onlyIf(() -> drivetrain.getDistanceError() < PREDICT_DISTANCE))
            ),
            Commands.waitUntil(() -> outtake.isDesiredCoralState(false))
        );
    }

    public static Command stow(RobotContainer robotContainer) {

        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            outtake.stopOuttake(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),  //TBD intake ramp pivot holds at intaking position for the duration of the match
            elevator.Elevator_Stow()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Stow"));

    }

    public static Command intakeCoralGround(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new SequentialCommandGroup(new InstantCommand()); //TBD
    }

    public static Command intakeCoralStation(RobotContainer robotContainer) {
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        TeleopPosSelector selector = robotContainer.getSelector();

        return new ParallelCommandGroup(
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            outtake.startIntaking(),
            elevator.Elevator_Stow(),
            rampPivot.Ramp_Intake_Pos()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Intake Coral Station")).andThen(new InstantCommand(() -> selector.hasCoralSIM = true));
    }

    public static Command intakeAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algaePivot.AlgaePivot_Horizontal(),
            algae.eatAlgae(),
            outtake.stopOuttake(),
            elevator.Elevator_Stow(),
            rampPivot.Ramp_Intake_Pos()

        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("intakeAlgae"));
    }

    public static Command L1Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            climb.Climb_Home(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),

            new SequentialCommandGroup(
                elevator.Elevator_L1(),
                Commands.waitUntil(() -> elevator.isElevatorInPos()),
                outtake.outtakeL1()
            ).withTimeout(1.0)
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L1 Scoring State"));
    }

    public static Command L2Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            new PrintCommand("uno"),
            rampPivot.Ramp_Intake_Pos(),
            new PrintCommand("dos"),
            climb.Climb_Retract(),
            new PrintCommand("tres"),
            algae.stopAlgae(),
            new PrintCommand("quadro"),
            algaePivot.AlgaePivot_Stow(),
            new PrintCommand("cinco"),

            new SequentialCommandGroup(
                elevator.Elevator_L2(),
                new PrintCommand("sies"),
                new waituntil(() -> elevator.isElevatorInPos()),
                new PrintCommand("siete"),
                outtake.startOuttake(),
                new PrintCommand("ocho")
            ).withTimeout(1.0)
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L2 Scoring State")).unless(()-> Robot.isSimulation());
    }

    public static Command L3Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),

            new SequentialCommandGroup(
                elevator.Elevator_L3(),
                Commands.waitUntil(() -> elevator.isElevatorInPos()),
                outtake.startOuttake()
            ).withTimeout(1.0)
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L3 scoring state"));
    }

    public static Command L4Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),

            new SequentialCommandGroup(
                elevator.Elevator_L4(),
                Commands.waitUntil(() -> elevator.isElevatorInPos()).withTimeout(2.0),
                outtake.outtakeL4(),
                new WaitCommand(0.1),
                elevator.Elevator_L4_Adj(),
                Commands.waitUntil(() -> elevator.isElevatorInPos()).withTimeout(2.0)
            ).withTimeout(3.0)
        )
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L4 Scoring State"));
    }

    public static Command LXCoral(RobotContainer robotContainer, int level){
        TeleopPosSelector selector = robotContainer.getSelector();

        switch(level){
            case 1:
                return L1Coral(robotContainer).andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

            case 2:
                return L2Coral(robotContainer).andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

            case 3:
                return L3Coral(robotContainer).andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

            case 4:
                return L4Coral(robotContainer).andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

            default:
                System.out.println("Invalid Coral Scoring Level!");
                return new InstantCommand();
        }
    }

    public static Command LXElevator(RobotContainer robotContainer, int level){
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new SequentialCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),
            Commands.waitUntil(() -> rampPivot.isSafeToRaiseElevator()),
            elevator.Elevator_LX(level)
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("L" + level+" Elevator Raise")).unless(()-> Robot.isSimulation());
    }

    public static Command processor(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            outtake.stopOuttake(),
            rampPivot.Ramp_Intake_Pos(),

            new SequentialCommandGroup(
                elevator.Elevator_Stow(),
                algaePivot.AlgaePivot_Punch(),
                algae.vomitAlgae()

            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("processor"));
    }

    public static Command netAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            outtake.stopOuttake(),
            rampPivot.Ramp_Intake_Pos(),

            new SequentialCommandGroup(
                elevator.Elevator_L4(),
                Commands.waitUntil(() -> elevator.getElevatorPositionRads() > 140.0),
                new ParallelCommandGroup(
                    algaePivot.AlgaePivot_NetAlgae(),
                    algae.vomitAlgae()
                )
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Net Algae"));
    }

    public static Command botAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            outtake.stopOuttake(),
            rampPivot.Ramp_Intake_Pos(),

            new SequentialCommandGroup(
                elevator.Elevator_BOT_BOT(), //TODO real height
                algaePivot.AlgaePivot_BotBot(),
                algae.eatAlgae()
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Bot Algae"));
    }

    public static Command topAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            outtake.stopOuttake(),
            rampPivot.Ramp_Intake_Pos(),

            new SequentialCommandGroup(
                elevator.Elevator_BOT_TOP().withTimeout(2), //TODO real height
                algaePivot.AlgaePivot_BotTop(),
                algae.eatAlgae()
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Top Algae"));
    }

    public static Command climb(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                outtake.rampEject(),
                algae.stopAlgae(),
                rampPivot.Ramp_Climb_Pos(),
                elevator.Elevator_Stow()
            ).alongWith(Commands.waitSeconds(1)),
            climb.Climb_Full()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Climb"));
    }
}
