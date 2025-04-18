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
import frc.robot.RobotContainer;
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
    public static Command driveToScore(RobotContainer robotContainer, PathPlannerPath pathToReadyPose, int level){
        CatzDrivetrain drivetrain = robotContainer.getCatzDrivetrain();
        CatzOuttake outtake = robotContainer.getCatzOuttake();

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPose, drivetrain, true, robotContainer, false).deadlineFor(
                new RepeatCommand(LXElevator(robotContainer, level).alongWith(new PrintCommand("elevavavava")).onlyIf(() -> drivetrain.getDistanceError() < DriveConstants.PREDICT_DISTANCE_SCORE))
            ),
            // new WaitUntilCommand(() -> !outtake.isDesiredCoralState(true)),
            LXCoral(robotContainer, level),
            new WaitUntilCommand(() -> outtake.isDesiredCoralState(true)),
            stow(robotContainer)
        );
    }

    public static Command driveToScore(RobotContainer robotContainer, PathPlannerPath pathToReadyPose, int level, double delay){
        CatzDrivetrain drivetrain = robotContainer.getCatzDrivetrain();
        CatzOuttake outtake = robotContainer.getCatzOuttake();

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPose, drivetrain, true, robotContainer, false).deadlineFor(
                new RepeatCommand(LXElevator(robotContainer, level).alongWith(new PrintCommand("elevavavava")).onlyIf(() -> drivetrain.getDistanceError() < DriveConstants.PREDICT_DISTANCE_SCORE))
            ),
            // new WaitUntilCommand(() -> !outtake.isDesiredCoralState(true)),
            new WaitCommand(delay),
            LXCoral(robotContainer, level),
            new WaitUntilCommand(() -> outtake.isDesiredCoralState(true)),
            stow(robotContainer)
        );
    }

    public static Command driveToScore(RobotContainer robotContainer, Supplier<PathPlannerPath> pathToReadyPoseSupplier, int level){

        CatzDrivetrain drivetrain = robotContainer.getCatzDrivetrain();

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPoseSupplier, drivetrain, false, robotContainer, false),
            LXElevator(robotContainer, level),
            moveScore(robotContainer, level),
            stow(robotContainer)
        );
    }

    public static Command moveScore(RobotContainer robotContainer, int level){
        CatzDrivetrain drivetrain = robotContainer.getCatzDrivetrain();
        TeleopPosSelector selector = robotContainer.getSelector();

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(()->selector.getMoveScorePath(), drivetrain, true, robotContainer, false),
            LXCoral(robotContainer, level)
        );
    }

    public static Command driveToCoralStation(RobotContainer robotContainer, PathPlannerPath path, Supplier<Boolean> waitForCoralSupplier){
        CatzDrivetrain drivetrain = robotContainer.getCatzDrivetrain();
        CatzOuttake outtake = robotContainer.getCatzOuttake();

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(path, drivetrain, false, robotContainer, false).deadlineFor(
                new RepeatCommand(intakeCoralStation(robotContainer).onlyIf(() -> drivetrain.getDistanceError() < DriveConstants.PREDICT_DISTANCE_INTAKE))
            ),
            Commands.waitUntil(() -> (waitForCoralSupplier.get() ? outtake.isDesiredCoralState(false) : true)).withTimeout(0.6)
        );
    }

    public static Command driveToCoralStation(RobotContainer robotContainer, Supplier<PathPlannerPath> pathSupplier, Supplier<Boolean> waitForCoralSupplier){
        return driveToCoralStation(robotContainer, pathSupplier.get(), waitForCoralSupplier);
    }

    //------------------------------------------------------------------------------------------------------------------------------------
    //
    //      MISC State Commands
    //
    //------------------------------------------------------------------------------------------------------------------------------------
    public static Command stow(RobotContainer robotContainer) {

        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

        return new ParallelCommandGroup(
            algae.stopAlgae(),
            outtake.stopOuttake(),
            algaePivot.AlgaePivot_Stow(),
            rampPivot.Ramp_Intake_Pos(),  //TBD intake ramp pivot holds at intaking position for the duration of the match
            intakeRollers.stopIntaking(),
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
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();
        CatzClimb climb = robotContainer.getCatzClimb();
        TeleopPosSelector selector = robotContainer.getSelector();

        return new ParallelCommandGroup(
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            outtake.startIntaking(),
            elevator.Elevator_Stow(),
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.intake()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Intake Coral Station")).andThen(new InstantCommand(() -> selector.hasCoralSIM = true));
    }

    public static Command intakeAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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
    public static Command L1Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            intakeRollers.stopIntaking(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),

            new SequentialCommandGroup(
                elevator.Elevator_L1(),
                Commands.print("elevator moving"),
                Commands.waitUntil(() -> elevator.isElevatorInPos()),
                Commands.print("elevator good"),
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
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

        return new ParallelCommandGroup(
            rampPivot.Ramp_Intake_Pos(),
            algae.stopAlgae(),
            algaePivot.AlgaePivot_Stow(),
            intakeRollers.stopIntaking(),

            new SequentialCommandGroup(
                elevator.Elevator_L2(),
                new waituntil(() -> elevator.isElevatorInPos()),
                new PrintCommand("outake l22"),
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
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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

    public static Command L4Coral(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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

    //------------------------------------------------------------------------------------------------------------------------------------
    //
    //      Preraise elevator
    //
    //------------------------------------------------------------------------------------------------------------------------------------
    public static Command LXElevator(RobotContainer robotContainer, int level){
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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
    public static Command processor(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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

    public static Command netAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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

    public static Command XAlgae(RobotContainer robotContainer, int level){
        switch(level){
            case 2:
            return topAlgae(robotContainer);

            case 4:
            return botAlgae(robotContainer);

            default:
            return new PrintCommand("Invalid Algae Level!");
        }
    }

    public static Command botAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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

    public static Command topAlgae(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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

    public static Command algaeStow(RobotContainer robotContainer) {

        CatzClimb climb = robotContainer.getCatzClimb();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaeRemover remover = robotContainer.getCatzAlgaeRemover();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

        return new ParallelCommandGroup(
            outtake.stopOuttake(),
            algaePivot.AlgaePivot_Punch(),
            rampPivot.Ramp_Intake_Pos(),  //TBD intake ramp pivot holds at intaking position for the duration of the match
            intakeRollers.stopIntaking(),
            elevator.Elevator_Coast_Stow(),
            remover.holdAlgae()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Stow"));
    }

    public static Command algaeGrndIntk(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzAlgaePivot algaePivot = robotContainer.getAlgaePivot();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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
    public static Command extendClimb(RobotContainer robotContainer) {
        CatzClimb climb = robotContainer.getCatzClimb();
        CatzAlgaeRemover algae = robotContainer.getCatzAlgaeRemover();
        CatzOuttake outtake = robotContainer.getCatzOuttake();
        CatzElevator elevator = robotContainer.getCatzElevator();
        CatzRampPivot rampPivot = robotContainer.getCatzRampPivot();
        CatzIntakeRollers intakeRollers = robotContainer.getIntakeRollers();

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
