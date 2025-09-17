package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.CatzAutonomous;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.CatzAlgaePivot;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.CatzAlgaeRemover;
import frc.robot.CatzSubsystems.CatzSuperstructure.Gamepiece;
import frc.robot.CatzSubsystems.CatzSuperstructure.LeftRight;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzElevator.*;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.ControllerLEDState;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;
import frc.robot.Utilities.OverrideSwitch;
import frc.robot.Utilities.Alert.AlertType;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


public class RobotContainer {
  public static final RobotContainer Instance = new RobotContainer();

  private final double SCORE_TRIGGER_THRESHHOLD = 0.8;

  private final double Drivetrain_Scale_Turn        = 0.5;
  private final double Drivetrain_Scale_Translation = 0.15;

  // ------------------------------------------------------------------------------------------------------------------
  // Drive Controller Declaration
  // -----------------------------------------------------------------------------------------------------------------
  private final CommandXboxController xboxDrv = new CommandXboxController(0);
  private final CommandXboxController xboxAux = new CommandXboxController(1);
  private final OverrideSwitch overrideHID = new OverrideSwitch(2);
  private final Trigger isClimbFullManualEnabled = overrideHID.auxSwitch(1);
  private final Trigger isElevatorFullManual = overrideHID.auxSwitch(2);
  private final Trigger isAlgaePivotFullManual = overrideHID.auxSwitch(3);
  private final Trigger isRampPivotFullManual = overrideHID.auxSwitch(4);
  private final Trigger climbMode = xboxAux.povLeft();

  private CommandXboxController xboxTest = new CommandXboxController(3);

  // -------------------------------------------------------------------------------------------------------------------
  // Alert Declaration
  // -------------------------------------------------------------------------------------------------------------------
  private final Alert disconnectedAlertDrive      = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert disconnectedAlertAux        = new Alert("Aux controller disconnected (port 1).", AlertType.kWarning);
  private final Alert disconnectedAlertOvverride  = new Alert("Override Switch disconnected (port 2)", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("Endgame Alert #1", 27.0);
  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("Endgame Alert #2", 10.0);

  // -------------------------------------------------------------------------------------------------------------------
  // Subsystem Declaration
  // ---------------------------------------------------------------------------------------------------------------------

  private RobotContainer() {
    // Drive And Aux Command Mapping
    configureBindings();

    // Endgame alert triggers
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9) // Rumble three times
);
  }

  // ---------------------------------------------------------------------------
  //
  //  Button mapping to commands
  //
  // ---------------------------------------------------------------------------

  private void configureBindings() {
    //---------------------------------------------------------------------------------------------------------------------
    // XBOX Drive
    //---------------------------------------------------------------------------------------------------------------------
    // Reset odometry
    DoublePressTracker.createTrigger(xboxDrv.button(8).and(xboxDrv.button(7))).onTrue(new InstantCommand(() -> {
      if(AllianceFlipUtil.shouldFlipToRed()){
        CatzRobotTracker.Instance.resetPose(new Pose2d(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), Rotation2d.k180deg));
      }else{
        CatzRobotTracker.Instance.resetPose(new Pose2d(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), new Rotation2d()));
      }
    }));

    // Climb
    Trigger climbMode = xboxDrv.povLeft();
    xboxDrv.b().onTrue(CatzSuperstructure.Instance.extendClimb());
    climbMode.toggleOnTrue(Commands.startEnd(()->CatzSuperstructure.Instance.setClimbOverride(()->true), ()->CatzSuperstructure.Instance.setClimbOverride(()->false)));

    //NBA
    xboxDrv.rightTrigger().onTrue(TeleopPosSelector.Instance.runToNearestBranch().alongWith(Commands.runOnce(()->CatzLED.Instance.setControllerState(ControllerLEDState.NBA)))
                                                               .onlyWhile(xboxDrv.rightTrigger())
                                                               .unless(()->CatzSuperstructure.isClimbEnabled())
                                                               .alongWith(CatzClimb.Instance.ClimbManualMode(()-> ((-xboxDrv.getRightTriggerAxis()-0.5)*2.0))
                                                               .onlyIf(()-> CatzSuperstructure.isClimbEnabled()))
    );
    xboxDrv.rightTrigger().onFalse(Commands.runOnce(()->CatzLED.Instance.setControllerState(ControllerLEDState.FULL_MANUAL))
                                           .unless(()-> CatzSuperstructure.isClimbEnabled())
                                           .alongWith(CatzClimb.Instance.CancelClimb())
    );

    // Left Right
    xboxDrv.leftBumper().onTrue(TeleopPosSelector.Instance.runLeftRight(LeftRight.LEFT).unless(()->CatzSuperstructure.isClimbEnabled()));
    xboxDrv.rightBumper().onTrue(TeleopPosSelector.Instance.runLeftRight(LeftRight.RIGHT).unless(()->CatzSuperstructure.isClimbEnabled()));

    //     // Manual Climb Control
    // xboxDrv.povUp().onTrue(CatzClimb.Instance.ClimbManualMode(()-> 0.4));
    // xboxDrv.povUp().onFalse(CatzClimb.Instance.CancelClimb());
    // xboxDrv.povDown().onTrue(CatzClimb.Instance.ClimbManualMode(()-> -1.0));
    // xboxDrv.povDown().onFalse(CatzClimb.Instance.CancelClimb());


    // Default driving
    Trigger escapeTrajectory = new Trigger(()->(xboxDrv.getLeftY() > 0.8));

    escapeTrajectory.onTrue(CatzDrivetrain.Instance.cancelTrajectory());

    CatzDrivetrain.Instance.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX()*Drivetrain_Scale_Translation, () -> xboxDrv.getLeftY()*Drivetrain_Scale_Translation, () -> xboxDrv.getRightX()*Drivetrain_Scale_Turn, CatzDrivetrain.Instance));
    //---------------------------------------------------------------------------------------------------------------------
    // XBOX AUX
    //------------------------------------------------------------------------------

    // Full Manual
    xboxAux.leftStick().and(xboxAux.rightStick()).onTrue(CatzElevator.Instance.elevatorFullManual(() -> -xboxAux.getLeftY()).alongWith(Commands.print("Elevator Manual"))); // TODO make an override button for
    isElevatorFullManual.onTrue(CatzElevator.Instance.elevatorFullManual(() -> -xboxAux.getLeftY()).alongWith(Commands.print("Elevator Manual"))); // TODO make an override button for
    xboxAux.leftStick().and(xboxAux.rightStick()).onTrue(CatzAlgaePivot.Instance.AlgaePivotFullManualCommand(()->xboxAux.getRightY()).alongWith(Commands.print("Algae Manual")));
    isAlgaePivotFullManual.onTrue(CatzAlgaePivot.Instance.AlgaePivotFullManualCommand(()->xboxAux.getRightY()).alongWith(Commands.print("Algae Manual")));
    isRampPivotFullManual.onTrue(CatzRampPivot.Instance.rampPivotManual(()->-xboxAux.getLeftY()).alongWith(Commands.print("Manual Ramp")));

    // Gamepiece Selection
    xboxAux.leftTrigger().onTrue(Commands.runOnce(()-> CatzSuperstructure.setChosenGamepiece(Gamepiece.CORAL)));
    xboxAux.rightTrigger().onTrue(Commands.runOnce(()-> CatzSuperstructure.setChosenGamepiece(Gamepiece.ALGAE)));

    // Scoring Action
    xboxAux.x().onTrue(CatzSuperstructure.Instance.intake().alongWith(Commands.print("INTAKE")));
    xboxAux.y().onTrue(CatzSuperstructure.Instance.LXCoral().alongWith(Commands.print("OUTTAKE L" + CatzSuperstructure.Instance.getLevel())));
    xboxAux.a().onTrue(CatzSuperstructure.Instance.stow().alongWith(Commands.print("STOWWW")));
    xboxAux.b().onTrue(CatzSuperstructure.Instance.algaeStow());


    // algae punch
    DoublePressTracker.createTrigger(xboxAux.x())
                      .onTrue(CatzAlgaePivot.Instance.AlgaePivot_Stow()
                                        .onlyIf(()->CatzSuperstructure.getChosenGamepiece() == Gamepiece.ALGAE)
                                        .alongWith(Commands.print("Algae Punch"))
    );

    isClimbFullManualEnabled.onTrue(Commands.print("CatzClimb.Instance full man"));

    // Scoring Level
    xboxAux.povRight().onTrue(Commands.runOnce(()-> {CatzSuperstructure.Instance.setLevel(1); SmartDashboard.putNumber("Reef Level", 1);}));
    xboxAux.povUp().onTrue(Commands.runOnce(() -> {CatzSuperstructure.Instance.setLevel(2); SmartDashboard.putNumber("Reef Level", 2);}));
    xboxAux.povLeft().onTrue(Commands.runOnce(() -> {CatzSuperstructure.Instance.setLevel(3); SmartDashboard.putNumber("Reef Level", 3);}));
    xboxAux.povDown().onTrue(Commands.runOnce(() -> {CatzSuperstructure.Instance.setLevel(4); SmartDashboard.putNumber("Reef Level", 4);}));
    //------------------------------------------------------------------------------------------------------------------------------
    //  XBOX test controls
    //------------------------------------------------------------------------------------------------------------------------------
    // xboxTest.povRight().toggleOnTrue(CatzAlgaePivot.Instance.AlgaePivot_BotTop().alongWith(Commands.print("pressed POV Right"))); //TBD

    // xboxTest.povUp().toggleOnTrue(CatzRampPivot.Instance.Ramp_Intake_Pos().alongWith(Commands.print("pressed POV Up"))); //TBD
    // xboxTest.povLeft().toggleOnTrue(CatzRampPivot.Instance.Ramp_Climb_Pos().alongWith(Commands.print("pressed POV Left")));
    xboxTest.a().onTrue(CatzSuperstructure.Instance.stow().alongWith(Commands.print("STOWWW")));

    xboxTest.povDown().toggleOnTrue(CatzAlgaePivot.Instance.AlgaePivot_NetAlgae().alongWith(Commands.print("pressed POV Right"))); //TBD
    xboxTest.povUp().toggleOnTrue(CatzAlgaePivot.Instance.AlgaePivot_Stow().alongWith(Commands.print("pressed POV Right"))); //TBD
    xboxTest.povRight().toggleOnTrue(CatzAlgaeRemover.Instance.eatAlgae().alongWith(Commands.print("pressed POV Right"))); //TBD
    xboxTest.povLeft().toggleOnTrue(CatzAlgaeRemover.Instance.vomitAlgae().alongWith(Commands.print("pressed POV Right"))); //TBD

    // xboxTest.x().onTrue(CatzAlgaePivot.Instance.AlgaePivot_Horizontal().alongWith(Commands.print("AL:KDJF:LAKDJFLK:ADJF:LKKJAD:FLKJ")));
    // xboxTest.y().onTrue(CatzAlgaePivot.Instance.AlgaePivot_Stow().alongWith(Commands.print("LAKJDFLKJALKJLKJFLSKDJLKKJSDLFKJLKKJLKKJFDKLJSLKJ")));

    // xboxTest.a().onTrue(CatzAlgaeRemover.Instance.eatAlgae().alongWith(Commands.print("ea")));
    // xboxTest.b().onTrue(CatzAlgaeRemover.Instance.vomitAlgae().alongWith(Commands.print("va")));

    // xboxTest.rightBumper().toggleOnTrue(CatzAlgaePivot.Instance.AlgaePivot_Stow().alongWith(Commands.print("stow")));
    // xboxTest.leftBumper().toggleOnTrue(CatzAlgaePivot.Instance.AlgaePivot_Horizontal().alongWith(Commands.print("eat")));
    // xboxTest.rightTrigger().onTrue(CatzAlgaeRemover.Instance.eatAlgae().alongWith(Commands.print("eating algae")));
    // xboxTest.leftTrigger().onTrue(CatzAlgaeRemover.Instance.vomitAlgae().alongWith(Commands.print("vomiting algae")));
    // xboxTest.rightStick().onTrue(CatzAlgaeRemover.Instance.stopAlgae().alongWith(Commands.print("stop algaeing")));

    // xboxTest.a().onTrue(outtake.startIntaking().alongWith(Commands.print("INTAKEKKKEEKKEKEKEING")));
    //xboxTest.b().onTrue(outtake.startOuttake().alongWith(Commands.print("INTAKEKKKEEKKEKEKEING")));
    //xboxTest.x().onTrue(outtake.stopOuttake().alongWith(Commands.print("INTAKEKKKEEKKEKEKEING")));

    // STOWING INTAKE RAMP FOR WHATEVER REASON
    //xboxTest.start().toggleOnTrue(CatzRampPivot.Instance.Ramp_Stow_Pos().alongWith(Commands.print("pressed start"))); //TBD
    // xboxTest.a().toggleOnTrue(CatzElevator.Instance.Elevator_Stow().alongWith(Commands.print("L1")));
    // xboxTest.b().toggleOnTrue(CatzElevator.Instance.Elevator_L2().alongWith(Commands.print("L2")));
    // xboxTest.x().toggleOnTrue(CatzElevator.Instance.Elevator_L3().alongWith(Commands.print("L3")));
    // xboxTest.y().toggleOnTrue(CatzElevator.Instance.Elevator_L4().alongWith(Commands.print("L4")));

    // xboxTest.rightStick().onTrue(CatzElevator.Instance.elevatorFullManual(()->xboxTest.getRightY()));

    xboxTest.rightStick().onTrue(CatzClimb.Instance.ClimbManualMode(() -> xboxTest.getLeftY()*5).alongWith(Commands.print("Using manual ramp pivot")));

    // Climb
    // xboxTest.back().and(xboxTest.b()).onTrue(CatzStateCommands.CatzClimb.Instance(this).alongWith(Commands.print("CatzClimb.Instance mode"))); // Setup Climb
    // xboxTest.back().and(xboxTest.x()).onTrue(CatzClimb.Instance.fullClimb());
    // xboxTest.back().and(xboxTest.start()).onTrue(CatzClimb.Instance.ClimbManualMode(()->0.0).alongWith(Commands.print("CatzClimb.Instance manual")));
    // // Manual Climb Control
    // xboxTest.back().and(xboxTest.leftStick()).onTrue(CatzClimb.Instance.ClimbManualMode(() -> xboxTest.getLeftY()).alongWith(Commands.print("Using manual CatzClimb.Instance")));

    // Climb SetPosition Control
    // xboxTest.y().toggleOnTrue(CatzClimb.Instance.Climb_Retract().alongWith(Commands.print("pressed y")));
    // xboxTest.x().toggleOnTrue(CatzClimb.Instance.Climb_Home().alongWith(Commands.print("pressed x")));
    // xboxTest.b().toggleOnTrue(CatzClimb.Instance.Climb_Full().alongWith(Commands.print("pressed b")));

    // xboxTest.x().toggleOnTrue(CatzRampPivot.Instance.Ramp_Stow().alongWith(Commands.print("pressed x"))); //TBD
    // xboxTest.b().toggleOnTrue(CatzRampPivot.Instance.Ramp_Climb().alongWith(Commands.print("pressed b"))); //TBD
    // xboxTest.a().toggleOnTrue(CatzRampPivot.Instance.Ramp_Intake().alongWith(Commands.print("pressed a")));

    // Manual Elevator Control
    Trigger leftJoystickTrigger = new Trigger(
      () -> Math.abs(xboxTest.getLeftY()) > 0.1);
    Trigger rightJoystickTrigger = new Trigger(
      () -> Math.abs(xboxTest.getRightY()) > 0.1);

    leftJoystickTrigger.onTrue(CatzRampPivot.Instance.rampPivotManual(() -> xboxTest.getLeftY()).alongWith(Commands.print("Using manual ramp pivot")));
    leftJoystickTrigger.onFalse(CatzRampPivot.Instance.rampPivotManual(()-> 0.0).alongWith(Commands.print("Nah - ramp motor")));

    //CatzClimb.Instance.ClimbManualMode(
    // rightJoystickTrigger.onTrue(CatzClimb.Instance.ClimbManualMode(() -> xboxTest.getRightY()).alongWith(Commands.print("Using manual CatzClimb.Instance")));
    // rightJoystickTrigger.onFalse(CatzClimb.Instance.ClimbManualMode(()-> 0.0).alongWith(Commands.print("Nah - CatzClimb.Instance motor")));

  }

  // ---------------------------------------------------------------------------
  //
  //  Misc methods
  //
  // ---------------------------------------------------------------------------
  /** Updates dashboard data. */
  // TODO if needed add values here
  public void updateDashboardOutputs() {}

  /** Updates the alerts. */
  public void updateAlerts() {}

  /** Creates a controller rumble command with specified rumble and controllers */
  public Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          xboxDrv.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          xboxAux.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          xboxDrv.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          xboxAux.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  public Command rumbleDrvController(double strength, double duration){
    return new SequentialCommandGroup(
      new InstantCommand(() ->xboxDrv.getHID().setRumble(RumbleType.kBothRumble, strength)),
      new WaitCommand(duration),
      new InstantCommand(() ->xboxDrv.getHID().setRumble(RumbleType.kBothRumble, 0.0))
    ).finallyDo(()->xboxDrv.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  public Command rumbleAuxController(double strength, double duration){
    return new SequentialCommandGroup(
      new InstantCommand(() ->xboxAux.getHID().setRumble(RumbleType.kBothRumble, strength)),
      new WaitCommand(duration),
      new InstantCommand(() ->xboxAux.getHID().setRumble(RumbleType.kBothRumble, 0.0))
    );
  }

  public Command rumbleDrvAuxController(double strength, double duration){
    return new ParallelCommandGroup(
      rumbleAuxController(strength, duration),
      rumbleDrvController(strength, duration)
    );
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    disconnectedAlertDrive.set(
        !DriverStation.isJoystickConnected(xboxDrv.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxDrv.getHID().getPort())
    );
    disconnectedAlertAux.set(
        !DriverStation.isJoystickConnected(xboxAux.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxAux.getHID().getPort())
    );
    disconnectedAlertOvverride.set(
      !overrideHID.getAuxSwitch(2)
    );

  }

  public Command getAutonomousCommand(){
    return CatzAutonomous.Instance.getCommand();
  }

  public CommandXboxController getXboxAux(){
    return xboxAux;
  }

  // ---------------------------------------------------------------------------
  //
  //      Subsystem getters
  //
  // ---------------------------------------------------------------------------
}
