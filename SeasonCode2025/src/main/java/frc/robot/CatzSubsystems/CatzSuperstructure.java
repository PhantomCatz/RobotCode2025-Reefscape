package frc.robot.CatzSubsystems;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.TeleopPosSelector;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.CatzAlgaePivot;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.CatzAlgaeRemover;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzIntakeRollers.CatzIntakeRollers;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.ControllerLEDState;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Utilities.VirtualSubsystem;
import frc.robot.Utilities.waituntil;
import lombok.Getter;
import lombok.Setter;

public class CatzSuperstructure extends VirtualSubsystem {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();

    //--------------------------------------------------------------------------------
    // Robot Operational State variables
    //--------------------------------------------------------------------------------
    static @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/ChosenGamepiece")
    private Gamepiece chosenGamepiece = Gamepiece.CORAL;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/Level")
    private int level = 1;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/CurrentRobotState")
    private RobotState currentRobotState = RobotState.STOW;

    @Getter @AutoLogOutput(key = "CatzSuperstructure/CurrentRobotAction")
    private RobotAction currentRobotAction = RobotAction.STOW;

    @Getter @AutoLogOutput(key = "CatzSuperstructure/IsClimbEnabled")
    private static boolean isClimbEnabled = false;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/CurrentCoralState")
    private static CoralState currentCoralState = CoralState.NOT_IN_OUTTAKE;

    @Getter @AutoLogOutput(key = "CatzSuperstructure/robotactionCommand")
    private Command robotActionCommand = Commands.none();

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/isScoring")
    private Supplier<Boolean> isScoring = () -> false;

    @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/canShoot")
    private Supplier<Boolean> canShoot = () -> false;

    //--------------------------------------------------------------------------------
    // RobotOperationalState Enums
    //--------------------------------------------------------------------------------
    public enum Gamepiece{
        CORAL,
        ALGAE
    }

    public enum CoralState{
        IN_OUTTAKE,
        NOT_IN_OUTTAKE,
        CORAL_ADJUSTING
    }

    public enum RobotState {
        STOW,
        INTAKE_CORAL_GROUND,
        INTAKE_CORAL_STATION,
        INTAKE_ALGAE_GROUND,
        GOBBLE_ALGAE,
        L1_CORAL,
        L2_CORAL,
        L3_CORAL,
        L4_CORAL,
        L1_AIMING,
        L2_AIMING,
        L3_AIMING,
        L4_AIMING,
        PROCESSOR,
        NET_ALGAE,
        BOT_ALGAE,
        TOP_ALGAE,
        CLIMBING,
        EXTENDING_CLIMB
    }

    public enum RobotAction {
        OUTTAKE,
        INTAKE,
        AIMING,
        INTAKE_GROUND,
        STOW,
        L4_AUTO_OUTTAKE
    }

    public enum LeftRight{
        LEFT(1),
        RIGHT(-1);

        public final int NUM;

        private LeftRight(int num){
          this.NUM  = num;
        }
    }

    private CatzSuperstructure() {
        this.level = 1;
    }

    // TODO: One driver controller should handle this, remove method
    // public void cycleGamePieceSelection() {
    //     if(chosenGamepiece == Gamepiece.CORAL) {
    //         chosenGamepiece = Gamepiece.ALGAE;
    //         System.out.println("Gamepiece: ALGAE");
    //     } else {
    //         chosenGamepiece = Gamepiece.CORAL;
    //         System.out.println("Gamepiece: CORAL");
    //     }
    // }

    //--------------------------------------------------------------------------------
    // Climb setting
    //--------------------------------------------------------------------------------
    public void setClimbOverride(BooleanSupplier isClimbEnabled) {
        CatzSuperstructure.isClimbEnabled = isClimbEnabled.getAsBoolean();
        if (isClimbEnabled()) {
            CatzLED.Instance.setControllerState(ControllerLEDState.CLIMB);
        }
        else {
            CatzLED.Instance.setControllerState(ControllerLEDState.FULL_MANUAL);
        }
        System.out.println("CLimb Enabled" + isClimbEnabled);
    }

    public Command extendClimb() {

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                CatzOuttake.Instance.stopOuttake(),
                CatzAlgaeRemover.Instance.stopAlgae(),
                CatzRampPivot.Instance.Ramp_Climb_Pos(),
                CatzIntakeRollers.Instance.stopIntaking()
            ).alongWith(Commands.waitSeconds(0.1)), //TBD TESITNG

            CatzClimb.Instance.extendClimb()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("EXTENDING CLIMB/////////////////////////////"));
    }

    //--------------------------------------------------------------------------------
    // Automated simple drivng and scoring commands
    //--------------------------------------------------------------------------------
    public Command driveToScore(PathPlannerPath pathToReadyPose, int level){

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPose, true, false).deadlineFor(
                new RepeatCommand(LXElevator(level).alongWith(new PrintCommand("elevavavava")).onlyIf(() -> CatzDrivetrain.Instance.getDistanceError() < DriveConstants.PREDICT_DISTANCE_SCORE))
            ),
            // new WaitUntilCommand(() -> !CatzOuttake.Instance.isDesiredCoralState(true)),
            LXCoral(level),
            new WaitUntilCommand(() -> CatzOuttake.Instance.isDesiredCoralState(true)),
            stow()
        );
    }

    public Command driveToScore(PathPlannerPath pathToReadyPose, int level, double delay){

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(pathToReadyPose, true, false).deadlineFor(
                new RepeatCommand(LXElevator(level).alongWith(new PrintCommand("elevavavava")).onlyIf(() -> CatzDrivetrain.Instance.getDistanceError() < DriveConstants.PREDICT_DISTANCE_SCORE))
            ),
            // new WaitUntilCommand(() -> !CatzOuttake.Instance.isDesiredCoralState(true)),
            new WaitCommand(delay),
            LXCoral(level),
            new WaitUntilCommand(() -> CatzOuttake.Instance.isDesiredCoralState(true)),
            stow()
        );
    }

     public Command driveToCoralStation(PathPlannerPath path, Supplier<Boolean> waitForCoralSupplier){

        return new SequentialCommandGroup(
            new TrajectoryDriveCmd(path, false, false).deadlineFor(
                new RepeatCommand(intakeCoralStation().onlyIf(() -> CatzDrivetrain.Instance.getDistanceError() < DriveConstants.PREDICT_DISTANCE_INTAKE))
            ),
            Commands.waitUntil(() -> (waitForCoralSupplier.get() ? CatzOuttake.Instance.isDesiredCoralState(false) : true)).withTimeout(0.6)
        );
    }

    //--------------------------------------------------------------------------------
    // One Drive Abstraction Commands
    //--------------------------------------------------------------------------------
    public Command scoreLevelXAutomated(int level){
        return Commands.sequence(
            Commands.runOnce(()-> {
                isScoring = () -> true;
                canShoot = () -> false;
                CatzSuperstructure.Instance.setLevel(level);
                Commands.runOnce(() -> CatzLED.Instance.setControllerState(ControllerLEDState.NBA));
            }),
            Commands.print("start score level"+level+" auto"),
            Commands.parallel(
                TeleopPosSelector.Instance.runToNearestBranch(),
                new WaitUntilCommand(() -> CatzDrivetrain.Instance.getDistanceError() < 0.1)
                .andThen(CatzElevator.Instance.Elevator_LX(level).asProxy())
            ),
            Commands.print("finish running"),
            new InstantCommand(() -> System.out.println("isScoring: " + isScoring.get())),
            new WaitUntilCommand(() -> CatzElevator.Instance.isElevatorInPos() && canShoot.get()),
            Commands.print("finish waiting"),
            CatzSuperstructure.Instance.ElevatorHeightShoot().asProxy()
        );
    }



    //--------------------------------------------------------------------------------
    // Mechanism Action Commands
    //--------------------------------------------------------------------------------
    public Command stow(){
        return new ParallelCommandGroup(
            CatzAlgaeRemover.Instance.stopAlgae(),
            CatzOuttake.Instance.stopOuttake(),
            CatzAlgaePivot.Instance.AlgaePivot_Stow(),
            CatzRampPivot.Instance.Ramp_Intake_Pos(),  //TODO? intake ramp pivot holds at intaking position for the duration of the match
            CatzIntakeRollers.Instance.stopIntaking(),
            CatzElevator.Instance.Elevator_Stow(),
            new InstantCommand(() -> currentRobotState = RobotState.STOW)
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Stow"));
    }

    public Command intakeCoralStation() {

        return new ParallelCommandGroup(
            CatzAlgaeRemover.Instance.stopAlgae(),
            CatzAlgaePivot.Instance.AlgaePivot_Stow(),
            CatzOuttake.Instance.startIntaking(),
            CatzElevator.Instance.Elevator_Stow(),
            CatzRampPivot.Instance.Ramp_Intake_Pos(),
            CatzIntakeRollers.Instance.intake()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Intake Coral Station")).andThen(new InstantCommand(() -> TeleopPosSelector.Instance.hasCoralSIM = true));
    }

    public Command intakeAlgae() {

        return new ParallelCommandGroup(
            CatzAlgaePivot.Instance.AlgaePivot_Horizontal(),
            CatzAlgaeRemover.Instance.eatAlgae(),
            CatzOuttake.Instance.stopOuttake(),
            CatzElevator.Instance.Elevator_Stow(),
            CatzRampPivot.Instance.Ramp_Intake_Pos(),
            CatzIntakeRollers.Instance.stopIntaking()

        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("intakeAlgae"));
    }

    public Command L1Coral() {

        return new ParallelCommandGroup(
            CatzRampPivot.Instance.Ramp_Intake_Pos(),
            CatzIntakeRollers.Instance.stopIntaking(),
            CatzAlgaeRemover.Instance.stopAlgae(),
            CatzAlgaePivot.Instance.AlgaePivot_Stow(),

            new SequentialCommandGroup(
                CatzElevator.Instance.Elevator_L1(),
                Commands.waitUntil(() -> CatzElevator.Instance.isElevatorInPos()).withTimeout(1.0),
                CatzOuttake.Instance.outtakeL1()
            )
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
        .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L1 Scoring State"));
    }

    public Command L2Coral() {

        return new ParallelCommandGroup(
            CatzRampPivot.Instance.Ramp_Intake_Pos(),
            CatzAlgaeRemover.Instance.stopAlgae(),
            CatzAlgaePivot.Instance.AlgaePivot_Stow(),
            CatzIntakeRollers.Instance.stopIntaking(),

            new SequentialCommandGroup(
                CatzElevator.Instance.Elevator_L2(),
                new waituntil(() -> CatzElevator.Instance.isElevatorInPos()).withTimeout(1.0), //huh
                CatzOuttake.Instance.startOuttake()
            )
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L2 Scoring State")).unless(()-> Robot.isSimulation());
    }

    public Command L3Coral() {

        return new ParallelCommandGroup(
            CatzRampPivot.Instance.Ramp_Intake_Pos(),
            CatzAlgaeRemover.Instance.stopAlgae(),
            CatzAlgaePivot.Instance.AlgaePivot_Stow(),
            CatzIntakeRollers.Instance.stopIntaking(),

            new SequentialCommandGroup(
                CatzElevator.Instance.Elevator_L3(),
                Commands.waitUntil(() -> CatzElevator.Instance.isElevatorInPos()).withTimeout(1.0),
                CatzOuttake.Instance.startOuttake()
            )
        )//.onlyIf(() -> CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE)
         .unless(()-> Robot.isSimulation()).alongWith(Commands.print("L3 scoring state"));
    }

    public Command L4Coral() {
        return new SequentialCommandGroup(
            CatzElevator.Instance.Elevator_L4(),
            Commands.waitUntil(() -> CatzElevator.Instance.isElevatorInPos()).withTimeout(2.0),
            CatzOuttake.Instance.outtakeL4(),
            new WaitCommand(0.1),
            CatzElevator.Instance.Elevator_L4_Adj(),
            Commands.waitUntil(() -> CatzElevator.Instance.isElevatorInPos()).withTimeout(2.0)
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("L4 Scoring State"));
    }

    //--------------------------------------------------------------------------------
    // Level Selector scoring
    //--------------------------------------------------------------------------------
    public Command LXCoral(int level){
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

    public Command LXCoral(){
        TeleopPosSelector selector = TeleopPosSelector.Instance;

        return new DeferredCommand(() -> {
            switch(level){
                case 1:
                currentRobotState = RobotState.L1_CORAL;

                return L1Coral().andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

                case 2:
                currentRobotState = RobotState.L2_CORAL;

                return L2Coral().andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

                case 3:
                currentRobotState = RobotState.L3_CORAL;

                return L3Coral().andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

                case 4:
                currentRobotState = RobotState.L4_CORAL;

                return L4Coral().andThen(new InstantCommand(()-> {selector.hasCoralSIM = false;}));

                default:
                System.out.println("Invalid Coral Scoring Level!");
                return new InstantCommand();
            }
        }, Set.of());
    }

    //--------------------------------------------------------------------------------
    // Level Selector Coral shooting
    //--------------------------------------------------------------------------------
    public Command ElevatorHeightShoot() {
        return new DeferredCommand(() -> {
            switch (CatzElevator.Instance.getElevatorTargetPosition()) {
                case PosL1:
                    return L1Shoot().alongWith(Commands.print("SHOOT L1")).andThen(new InstantCommand(()-> {TeleopPosSelector.Instance.hasCoralSIM = false;}));
                case PosL2:
                    return L2Shoot().alongWith(Commands.print("SHOOT L2")).andThen(new InstantCommand(()-> {TeleopPosSelector.Instance.hasCoralSIM = false;}));

                case PosL3:
                    return L3Shoot().alongWith(Commands.print("SHOOT L3")).andThen(new InstantCommand(()-> {TeleopPosSelector.Instance.hasCoralSIM = false;}));

                case PosL4:
                    return L4Shoot().alongWith(Commands.print("SHOOT L4")).andThen(new InstantCommand(()-> {TeleopPosSelector.Instance.hasCoralSIM = false;}));

                default:
                    return new PrintCommand("Invalid Level: " + level);
            }
        }, Set.of());
    }

    public Command LXShoot(int level){
        return new DeferredCommand(() -> {
            switch(level){
                case 1:
                    return L1Shoot().andThen(new InstantCommand(()-> {TeleopPosSelector.Instance.hasCoralSIM = false;}));

                case 2:
                    return L2Shoot().andThen(new InstantCommand(()-> {TeleopPosSelector.Instance.hasCoralSIM = false;}));

                case 3:
                    return L3Shoot().andThen(new InstantCommand(()-> {TeleopPosSelector.Instance.hasCoralSIM = false;}));

                case 4:
                    return L4Shoot().andThen(new InstantCommand(()-> {TeleopPosSelector.Instance.hasCoralSIM = false;}));

                default:
                    return new PrintCommand("Invalid Level: " + level);
            }
        }, Set.of());
    }

    public Command L1Shoot(){
        return CatzOuttake.Instance.outtakeL1();
    }

    public Command L2Shoot(){
        return CatzOuttake.Instance.startOuttake();
    }

    public Command L3Shoot(){
        return CatzOuttake.Instance.startOuttake();
    }

    /**
     *
     * @return Command to eject in L4 with elevator adjust. Does not wait for the elevator to go up to adjust height.
     */
    public Command L4Shoot(){
        return Commands.sequence(
            CatzOuttake.Instance.outtakeL4(),
            new WaitCommand(0.1),
            CatzElevator.Instance.Elevator_L4_Adj()
        );
    }

    public Command intake(){
        return new DeferredCommand(() -> {
            if(chosenGamepiece == Gamepiece.CORAL) {
                currentRobotState = RobotState.INTAKE_CORAL_STATION;
                return intakeCoralStation();
            } else {
                switch (level) {
                    case 2:
                        currentRobotState = RobotState.TOP_ALGAE;
                        System.out.println("TOP algae");
                        return botAlgae();
                    case 4:
                        currentRobotState = RobotState.BOT_ALGAE;
                        System.out.println("BOT algae");
                        return topAlgae();
                    default:
                        return Commands.none();
                }
        }}, Set.of());
    }

    public Command LXElevator(int level){

        return new SequentialCommandGroup(
            // new PrintCommand("not yet"),
            // Commands.waitUntil(() -> CatzRampPivot.Instance.isSafeToRaiseElevator()),
            // new PrintCommand("safe to raise CatzElevator.Instance!!"),
            Commands.print("LXElevator run"),
            CatzElevator.Instance.Elevator_LX(level)
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("L" + level+" CatzElevator.Instance Raise")).unless(()-> Robot.isSimulation());
    }

    //--------------------------------------------------------------------------------
    // Algae Level Selector and Scoring
    //--------------------------------------------------------------------------------

    public Command botAlgae() {

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                CatzOuttake.Instance.stopOuttake(),
                CatzRampPivot.Instance.Ramp_Intake_Pos(),
                CatzIntakeRollers.Instance.stopIntaking(),
                CatzAlgaePivot.Instance.Algae_Transition_Bot(),
                CatzElevator.Instance.Elevator_Bot_Transition()
            ),
            new SequentialCommandGroup(
                Commands.waitSeconds((0.2)),
                CatzElevator.Instance.Elevator_BOT_BOT(), //TODO real height
                CatzAlgaePivot.Instance.AlgaePivot_BotBot(),
                CatzAlgaeRemover.Instance.eatAlgae()
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Bot Algae"));
    }

    public Command topAlgae() {

        return new ParallelCommandGroup(
            CatzOuttake.Instance.stopOuttake(),
            CatzRampPivot.Instance.Ramp_Intake_Pos(),
            CatzIntakeRollers.Instance.stopIntaking(),
            CatzElevator.Instance.Elevator_BOT_TOP(),

            new SequentialCommandGroup(
                CatzAlgaePivot.Instance.AlgaePivot_BotTop().alongWith(Commands.waitSeconds(0.2)),
                CatzAlgaeRemover.Instance.eatAlgae()
            )
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Top Algae"));
    }

    public Command XAlgae(int level){
        switch(level){
            case 2:
            return topAlgae();

            case 4:
            return botAlgae();

            default:
            return new PrintCommand("Invalid Algae Level!");
        }
    }

    public Command algaeStow() {
        return new ParallelCommandGroup(
            CatzOuttake.Instance.stopOuttake(),
            CatzAlgaePivot.Instance.AlgaePivot_Punch(),
            CatzRampPivot.Instance.Ramp_Intake_Pos(),  //TBD intake ramp pivot holds at intaking position for the duration of the match
            CatzIntakeRollers.Instance.stopIntaking(),
            CatzElevator.Instance.Elevator_Coast_Stow(),
            CatzAlgaeRemover.Instance.holdAlgae()
        ).unless(()-> Robot.isSimulation()).alongWith(Commands.print("Stow"));
    }

    //--------------------------------------------------------------------------------
    // Autonomous specific commands
    //--------------------------------------------------------------------------------
    public Command scoreInAuto(int level){
        return Commands.sequence
        (
            CatzElevator.Instance.Elevator_LX(level),
            Commands.waitUntil(() -> readyToScoreAuton()),
            LXShoot(level),
            Commands.waitUntil(() -> CatzOuttake.Instance.isDesiredCoralState(true))
        );
    }

    private boolean readyToScoreAuton(){
        if(Robot.isSimulation()){
            return CatzDrivetrain.Instance.isRobotAtPoseChoreo();
        }else{
            return CatzDrivetrain.Instance.isRobotAtPoseChoreo() && CatzElevator.Instance.isElevatorInPos();
        }
    }


    @Override
    public void periodic() {

        //----------------------------------------------------------------------------------
        // Logging
        //----------------------------------------------------------------------------------
        getChosenGamepiece();
        getLevel();
        getCurrentRobotState();
        getCurrentRobotAction();
        getCurrentCoralState();
        isClimbEnabled();

    }


}
