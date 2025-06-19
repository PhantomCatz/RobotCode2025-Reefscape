//------------------------------------------------------------------------------------
// 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project. 
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.;
import frc.robot.TeleopPosSelector;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.CatzAlgaePivot;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.CatzAlgaeRemover;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;
import frc.robot.CatzSubsystems.CatzIntakeRollers.CatzIntakeRollers;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Utilities.waituntil;

public class CatzStateCommands {

    //------------------------------------------------------------------------------------------------------------------------------------
    //
    //      Autonomous State COmmands
    //
    //------------------------------------------------------------------------------------------------------------------------------------
    public static Command driveToScore(PathPlannerPath pathToReadyPose, int level){
        CatzDrivetrain drivetrain = CatzDrivetrain.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPose, drivetrain, true, false).deadlineFor(
                new RepeatCommand(LXElevator(level).alongWith(new PrintCommand("elevavavava")).onlyIf(() -> drivetrain.getDistanceError() < DriveConstants.PREDICT_DISTANCE_SCORE))
            ),
            // new WaitUntilCommand(() -> !outtake.isDesiredCoralState(true)),
            LXCoral(level),
            new WaitUntilCommand(() -> outtake.isDesiredCoralState(true)),
            stow()
        );
    }

    public static Command driveToScore(PathPlannerPath pathToReadyPose, int level, double delay){
        CatzDrivetrain drivetrain = CatzDrivetrain.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPose, drivetrain, true, false).deadlineFor(
                new RepeatCommand(LXElevator(level).alongWith(new PrintCommand("elevavavava")).onlyIf(() -> drivetrain.getDistanceError() < DriveConstants.PREDICT_DISTANCE_SCORE))
            ),
            // new WaitUntilCommand(() -> !outtake.isDesiredCoralState(true)),
            new WaitCommand(delay),
            LXCoral(level),
            new WaitUntilCommand(() -> outtake.isDesiredCoralState(true)),
            stow()
        );
    }

    public static Command driveToScore(Supplier<PathPlannerPath> pathToReadyPoseSupplier, int level){

        CatzDrivetrain drivetrain = CatzDrivetrain.Instance;

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPoseSupplier, drivetrain, false, false),
            LXElevator(level),
            moveScore(level),
            stow()
        );
    }

    public static Command moveScore(int level){
        CatzDrivetrain drivetrain = CatzDrivetrain.Instance;
        TeleopPosSelector selector = TeleopPosSelector.Instance;

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(()->selector.getMoveScorePath(), drivetrain, true, false),
            LXCoral(level)
        );
    }

    public static Command driveToCoralStation(PathPlannerPath path, Supplier<Boolean> waitForCoralSupplier){
        CatzDrivetrain drivetrain = CatzDrivetrain.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(path, drivetrain, false, false).deadlineFor(
                new RepeatCommand(intakeCoralStation().onlyIf(() -> drivetrain.getDistanceError() < DriveConstants.PREDICT_DISTANCE_INTAKE))
            ),
            Commands.waitUntil(() -> (waitForCoralSupplier.get() ? outtake.isDesiredCoralState(false) : true)).withTimeout(0.6)
        );
    }

    public static Command driveToCoralStation(Supplier<PathPlannerPath> pathSupplier, Supplier<Boolean> waitForCoralSupplier){
        return driveToCoralStation(pathSupplier.get(), waitForCoralSupplier);
    }

    //------------------------------------------------------------------------------------------------------------------------------------
    //
    //      MISC State Commands
    //
    //------------------------------------------------------------------------------------------------------------------------------------
    public static Command stow() {

        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            algae.stopAlgae(),
            outtake.stopOuttake(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),  //TBD intake ramp pivot holds at intaking position for the duration of the match
            intakeRollers.stopIntaking(),
            elevator.Elevator_Stow()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Stow"));

    }

    public static Command intakeCoralStation() {
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;
        CatzClimb climb = CatzClimb.Instance;
        TeleopPosSelector selector = TeleopPosSelector.Instance;

        return new ParallelCommandGroup(
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            outtake.startIntaking(),
            elevator.Elevator_Stow(),
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.intake()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Intake Coral Station")).andThen(new InstantCommand(() -> selector.hasCoralSIM = true));
    }

    public static Command intakeAlgae() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            algaePivot.AlgaePivot_Horizontal(),
            algae.eatAlgae(),
            outtake.stopOuttake(),
            elevator.Elevator_Stow(),
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.stopIntaking()

        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("intakeAlgae"));
    }

    //------------------------------------------------------------------------------------------------------------------------------------
    //
    //      Scoring State COmmands
    //
    //------------------------------------------------------------------------------------------------------------------------------------
    public static Command L1Coral() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.stopIntaking(),
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

    public static Command L2Coral() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            intakeRollers.stopIntaking(),

            new SequentialCommandGroup(
                elevator.Elevator_L2(),
                new waituntil(() -> elevator.isElevatorInPos()),
                outtake.startOuttake()
            ).withTimeout(1.0)
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L2 Scoring State")).unless(()-> Robot.isSimulation());
    }

    public static Command L3Coral() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            intakeRollers.stopIntaking(),

            new SequentialCommandGroup(
                elevator.Elevator_L3(),
                Commands.waitUntil(() -> elevator.isElevatorInPos()),
                outtake.startOuttake()
            ).withTimeout(1.0)
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L3 scoring state"));
    }

    public static Command L4Coral() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            intakeRollers.stopIntaking(),

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

    public static Command LXCoral(, int level){
        TeleopPosSelector selector = TeleopPosSelector.Instance;

        switch(level){
            case 1:
                return L1Coral().andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

            case 2:
                return L2Coral().andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

            case 3:
                return L3Coral().andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

            case 4:
                return L4Coral().andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

            default:
                System.out.println("Invalid Coral Scoring Level!");
                return new InstantCommand();
        }
    }

    //------------------------------------------------------------------------------------------------------------------------------------
    //
    //      Preraise elevator
    //
    //------------------------------------------------------------------------------------------------------------------------------------
    public static Command LXElevator(, int level){
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new SequentialCommandGroup(
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.stopIntaking(),
            // new PrintCommand("not yet"),
            // Commands.waitUntil(() -> rampPivot.isSafeToRaiseElevator()),
            // new PrintCommand("safe to raise elevator!!"),
            elevator.Elevator_LX(level)
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("L" + level+" Elevator Raise")).unless(()-> Robot.isSimulation());
    }

    //------------------------------------------------------------------------------------------------------------------------------------
    //
    //      Algae State COmmands
    //
    //------------------------------------------------------------------------------------------------------------------------------------
    public static Command processor() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            outtake.stopOuttake(),
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.stopIntaking(),

            new SequentialCommandGroup(
                elevator.Elevator_Stow(),
                algaePivot.AlgaePivot_BotBot(),
                algae.vomitAlgae()

            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("processor"));
    }

    public static Command netAlgae() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            outtake.stopOuttake(),
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.stopIntaking(),

            new SequentialCommandGroup(
                elevator.Elevator_L4(),
                Commands.waitUntil(() -> elevator.getElevatorPositionInch() > 50.0),
                new ParallelCommandGroup(
                    algaePivot.AlgaePivot_NetAlgae(),
                    algae.eatAlgae()
                )
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Net Algae"));
    }

    public static Command XAlgae(, int level){
        switch(level){
            case 2:
            return topAlgae();

            case 4:
            return botAlgae();

            default:
            return new PrintCommand("Invalid Algae Level!");
        }
    }

    public static Command botAlgae() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                outtake.stopOuttake(),
                rampPivot.Ramp_Intake_Pos(),
                intakeRollers.stopIntaking(),
                algaePivot.Algae_Transition_Bot(),
                elevator.Elevator_Bot_Transition()
            ),
            new SequentialCommandGroup(
                Commands.waitSeconds((0.2)),
                elevator.Elevator_BOT_BOT(), //TODO real height
                algaePivot.AlgaePivot_BotBot(),
                algae.eatAlgae()
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Bot Algae"));
    }

    public static Command topAlgae() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            outtake.stopOuttake(),
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.stopIntaking(),
            elevator.Elevator_BOT_TOP(),

            new SequentialCommandGroup(
                algaePivot.AlgaePivot_BotTop().alongWith(Commands.waitSeconds(0.2)),
                algae.eatAlgae()
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Top Algae"));
    }

    public static Command algaeStow() {

        CatzClimb climb = CatzClimb.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaeRemover remover = CatzAlgaeRemover.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            outtake.stopOuttake(),
            algaePivot.AlgaePivot_Punch(),
            rampPivot.Ramp_Intake_Pos(),  //TBD intake ramp pivot holds at intaking position for the duration of the match
            intakeRollers.stopIntaking(),
            elevator.Elevator_Coast_Stow(),
            remover.holdAlgae()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Stow"));
    }

    public static Command algaeGrndIntk() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzAlgaePivot algaePivot = CatzAlgaePivot.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new ParallelCommandGroup(
            outtake.stopOuttake(),
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.stopIntaking(),
            elevator.Elevator_Stow(),

            new SequentialCommandGroup(
                algaePivot.AlgaePivot_GrndIntake().alongWith(Commands.waitSeconds(0.2)),
                algae.vomitAlgae() //needs to spin other way
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Algae Ground Intake"));
    }

    //------------------------------------------------------------------------------------------------------------------------------------
    //
    //      Climb Commands
    //
    //------------------------------------------------------------------------------------------------------------------------------------
    public static Command extendClimb() {
        CatzClimb climb = CatzClimb.Instance;
        CatzAlgaeRemover algae = CatzAlgaeRemover.Instance;
        CatzOuttake outtake = CatzOuttake.Instance;
        CatzElevator elevator = CatzElevator.Instance;
        CatzRampPivot rampPivot = CatzRampPivot.Instance;
        CatzIntakeRollers intakeRollers = CatzIntakeRollers.Instance;

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                outtake.stopOuttake(),
                algae.stopAlgae(),
                rampPivot.Ramp_Climb_Pos(),
                intakeRollers.stopIntaking()
            ).alongWith(Commands.waitSeconds(0.1)), //TBD TESITNG

            climb.extendClimb()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("EXTENDING CLIMB/////////////////////////////"));
    }

}
