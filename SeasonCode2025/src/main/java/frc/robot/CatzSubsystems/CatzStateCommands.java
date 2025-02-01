// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.CatzAlgaeRemover;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator.ElevatorPosition;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;

public class CatzStateCommands {


    public static Command stow(RobotContainer robotContainer) {

        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            algae.stopAlgae(),
            outtake.stopOuttake(),
            elevator.setTargetPositionCommand(ElevatorPosition.PosL1Home)
        );

    }

    public static Command intakeCoralGround(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new SequentialCommandGroup(null); //TBD
    }

    public static Command intakeCoralStation(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            algae.stopAlgae(),
            outtake.startIntaking(),
            elevator.setTargetPositionCommand(ElevatorPosition.PosL1Home)
        );
    }

    public static Command intakeAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            algae.eatAlgae(),
            outtake.stopOuttake(),
            elevator.setTargetPositionCommand(ElevatorPosition.PosL1Home)
        );
    }

    public static Command L1Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            algae.stopAlgae(),

            new SequentialCommandGroup(
                elevator.setTargetPositionCommand(ElevatorPosition.PosL1Home),
                outtake.startOuttake()
            )
        );
    }

    public static Command L2Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            algae.stopAlgae(),

            new SequentialCommandGroup(
                elevator.setTargetPositionCommand(ElevatorPosition.PosL2),
                outtake.startOuttake()
            )
        );
    }

    public static Command L3Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            algae.stopAlgae(),

            new SequentialCommandGroup(
                elevator.setTargetPositionCommand(ElevatorPosition.PosL3),
                outtake.startOuttake()
            )
        );
    }

    public static Command L4Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            algae.stopAlgae(),

            new SequentialCommandGroup(
                elevator.setTargetPositionCommand(ElevatorPosition.PosL4),
                outtake.startOuttake()
            )
        );
    }

    public static Command processor(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            outtake.stopOuttake(),

            new SequentialCommandGroup(
                elevator.setTargetPositionCommand(ElevatorPosition.PosL1Home),
                algae.vomitAlgae()

            )
        );
    }

    public static Command botAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            outtake.stopOuttake(),

            new SequentialCommandGroup(
                elevator.setTargetPositionCommand(ElevatorPosition.PosL2), //TODO real height
                algae.eatAlgae()
            )
        );
    }

    public static Command topAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new ParallelCommandGroup(
            climb.Climb_Stow(),
            outtake.stopOuttake(),

            new SequentialCommandGroup(
                elevator.setTargetPositionCommand(ElevatorPosition.PosL3), //TODO real height
                algae.eatAlgae()
            )
        );
    }

    public static Command climb(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                outtake.stopOuttake(),
                algae.stopAlgae(),
                elevator.setTargetPositionCommand(ElevatorPosition.PosL1Home)
            ),
            climb.Climb_Full()
        );
    }
}
