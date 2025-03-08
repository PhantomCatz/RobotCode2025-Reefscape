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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            // climb.Climb_Retract(),
            new PrintCommand("intakkkkkekekekekkeke"),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            outtake.startIntaking(),
            elevator.Elevator_Stow(),
            rampPivot.Ramp_Intake_Pos()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Intake Coral Station"));
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
                Commands.waitUntil(() -> elevator.isElevatorInPosition()),
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
            rampPivot.Ramp_Intake_Pos(),
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),

            new SequentialCommandGroup(
                elevator.Elevator_L2(),
                Commands.waitUntil(() -> elevator.isElevatorInPosition()),
                outtake.startOuttake()
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
                Commands.waitUntil(() -> elevator.isElevatorInPosition()),
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
                Commands.waitUntil(() -> elevator.isElevatorInPosition()).withTimeout(0.6).alongWith(Commands.print("/////////L4?////////")),
                outtake.outtakeL4(),
                new WaitCommand(0.06),
                elevator.Elevator_L4_Adj()
            ).withTimeout(2.0)
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L4 Scoring State"));
    }

    public static Command L1Elevator(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),
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
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),
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
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),
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
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();

        return new ParallelCommandGroup(
            climb.Climb_Retract(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),
            algae.stopAlgae(),
            elevator.Elevator_L4()
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
            ),
            climb.Climb_Full()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Climb"));
    }
}
