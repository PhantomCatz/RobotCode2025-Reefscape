// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.CatzAlgaePivot;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.CatzAlgaeRemover;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;

public class CatzStateCommands {


    public static Command stow(RobotContainer robotContainer) {

        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            outtake.stopOuttake(),
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
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            // climb.Climb_Retract(),
            algae.stopAlgae(),
            outtake.startIntaking(),
            elevator.Elevator_Stow()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Intake Coral Station"));
    }

    public static Command intakeAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.eatAlgae(),
            outtake.stopOuttake(),
            elevator.Elevator_Stow()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("intakeAlgae"));
    }

    public static Command L1Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Home(),
            algae.stopAlgae(),

            new SequentialCommandGroup(
                elevator.Elevator_L1(),
                Commands.waitUntil(() -> elevator.isElevatorInPosition()),
                outtake.outtakeL1()
            )
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L1 Scoring State"));
    }

    public static Command L2Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),

            new SequentialCommandGroup(
                elevator.Elevator_L2(),
                Commands.waitUntil(() -> elevator.isElevatorInPosition()),
                outtake.startOuttake()
            )
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L2 Scoring State")).unless(()-> Robot.isSimulation());
    }

    public static Command L3Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),

            new SequentialCommandGroup(
                elevator.Elevator_L3(),
                Commands.waitUntil(() -> elevator.isElevatorInPosition()),
                outtake.startOuttake()
            )
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L3 scoring state"));
    }

    public static Command L4Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            new PrintCommand("L4 Score!!!!!"),
            climb.Climb_Retract(),
            algae.stopAlgae(),

            new SequentialCommandGroup(
                elevator.Elevator_L4(),
                Commands.waitUntil(() -> elevator.isElevatorInPosition()).withTimeout(0.8).alongWith(Commands.print("/////////L4?////////")),
                outtake.outtakeL4().alongWith(Commands.print("/????????????outake???????????")),
                Commands.waitSeconds(0.5),
                elevator.Elevator_L4_Adj()
            )
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L4 Scoring State"));
    }

    public static Command L1Elevator(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            elevator.Elevator_L1()
        )
        //.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L1 Elevator")).unless(()-> Robot.isSimulation());
    }

    public static Command L2Elevator(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            elevator.Elevator_L2()
        )
        //.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L2 Scoring State")).unless(()-> Robot.isSimulation());
    }

    public static Command L3Elevator(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            elevator.Elevator_L3()
        )
        //.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L3 Scoring State")).unless(()-> Robot.isSimulation());
    }

    public static Command L4Elevator(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();

        return new ParallelCommandGroup(
            elevator.Elevator_L4(),
            climb.Climb_Retract(),
            algaePivot.AlgaePivot_Stow(),
            algae.stopAlgae()
        )
        //.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L4 Scoring State")).unless(()-> Robot.isSimulation());
    }

    public static Command processor(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            outtake.stopOuttake(),

            new SequentialCommandGroup(
                elevator.Elevator_Stow(),
                algaePivot.AlgaePivot_Stow(),
                algae.vomitAlgae()

            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("processor"));
    }

    public static Command botAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            outtake.stopOuttake(),

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

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            outtake.stopOuttake(),

            new SequentialCommandGroup(
                elevator.Elevator_BOT_TOP(), //TODO real height
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
                outtake.stopOuttake(),
                algae.stopAlgae(),
                rampPivot.Ramp_Climb(),
                elevator.Elevator_Stow()
            ),
            climb.Climb_Full()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Climb"));
    }
}
