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
import frc.robot.CatzSubsystems.CatzStateCommands;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot.CatzAlgaePivot;
import frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.CatzAlgaeRemover;
import frc.robot.CatzSubsystems.CatzSuperstructure.Gamepiece;
import frc.robot.CatzSubsystems.CatzSuperstructure.LeftRight;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.CatzVision;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.VisionIOLimelight;
import frc.robot.CatzSubsystems.CatzElevator.*;
import frc.robot.CatzSubsystems.CatzIntakeRollers.CatzIntakeRollers;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.ControllerLEDState;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.TeleopPosSelector.CancellationSource;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.DoublePressTracker;
import frc.robot.Utilities.OverrideSwitch;
import frc.robot.Utilities.Alert.AlertType;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


public class RobotContainer {

  private final double SCORE_TRIGGER_THRESHHOLD = 0.8;

  // -------------------------------------------------------------------------------------------------------------------
  // Subsystem Declaration
  // -------------------------------------------------------------------------------------------------------------------
  // Primary subsystem declaration
  private static CatzDrivetrain drive = new CatzDrivetrain();

  // Assistance Subsystem declaration
  private CatzLED led = CatzLED.getInstance();
  private CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private CatzVision vision = new CatzVision(new VisionIOLimelight("limelight-gyoza"),
                                             new VisionIOLimelight("limelight-tempura"));
  private CatzOuttake outtake;
  private CatzElevator elevator = new CatzElevator();
  private CatzClimb climb = new CatzClimb();
  private CatzAlgaeRemover algaeRemover = new CatzAlgaeRemover();
  private CatzAlgaePivot algaePivot = new CatzAlgaePivot();
  private CatzRampPivot rampPivot = new CatzRampPivot();
  private CatzIntakeRollers intakeRollers = new CatzIntakeRollers();
  private CatzSuperstructure superstructure = new CatzSuperstructure(this);

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
  private TeleopPosSelector selector;

  // -------------------------------------------------------------------------------------------------------------------
  // Alert Declaration
  // -------------------------------------------------------------------------------------------------------------------
  private final Alert disconnectedAlertDrive      = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert disconnectedAlertAux        = new Alert("Aux controller disconnected (port 1).", AlertType.kWarning);
  private final Alert disconnectedAlertOvverride  = new Alert("Override Switch disconnected (port 2)", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("Endgame Alert #1", 27.0);
  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("Endgame Alert #2", 10.0);

  // -------------------------------------------------------------------------------------------------------------------
  // Auto Declaration
  // ---------------------------------------------------------------------------------------------------------------------

  private CatzAutonomous auto;

  public RobotContainer() {
    outtake = new CatzOuttake(this);
    selector = new TeleopPosSelector(xboxAux, this);
    auto = new CatzAutonomous(this);


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
        robotTracker.resetPose(new Pose2d(robotTracker.getEstimatedPose().getTranslation(), Rotation2d.k180deg));
      }else{
        robotTracker.resetPose(new Pose2d(robotTracker.getEstimatedPose().getTranslation(), new Rotation2d()));
      }
    }));

    // Climb
    Trigger climbMode = xboxDrv.povLeft();
    xboxDrv.b().onTrue(CatzStateCommands.extendClimb(this));
    climbMode.toggleOnTrue(Commands.startEnd(()->superstructure.setClimbOverride(()->true), ()->superstructure.setClimbOverride(()->false)));

    //NBA
    xboxDrv.rightTrigger().onTrue(selector.runToNearestBranch().alongWith(Commands.runOnce(()->led.setControllerState(ControllerLEDState.NBA)))
                                                               .unless(()->CatzSuperstructure.isClimbEnabled())
                                                               .alongWith(climb.ClimbManualMode(()-> ((-xboxDrv.getRightTriggerAxis()-0.5)*2.0))
                                                               .onlyIf(()-> CatzSuperstructure.isClimbEnabled()))
    );
    xboxDrv.rightTrigger().onFalse(selector.cancelCurrentDrivetrainCommand(CancellationSource.NBA)
                                           .alongWith(Commands.runOnce(()->led.setControllerState(ControllerLEDState.FULL_MANUAL)))
                                           .alongWith(climb.CancelClimb())
    );

    // NBA Barge
    xboxDrv.leftTrigger().onTrue(selector.runToNetCommand()
                                         .alongWith(Commands.runOnce(()->led.setControllerState(ControllerLEDState.NBA)))
                                         .unless(()->CatzSuperstructure.isClimbEnabled())
                                         .alongWith(climb.ClimbManualMode(()-> ((xboxDrv.getLeftTriggerAxis()-0.5)*2.0))
                                         .onlyIf(()-> CatzSuperstructure.isClimbEnabled()))
    );
    xboxDrv.leftTrigger().onFalse(selector.cancelCurrentDrivetrainCommand(CancellationSource.NET)
                                          .alongWith(Commands.runOnce(()->led.setControllerState(ControllerLEDState.FULL_MANUAL)))
                                          .alongWith(climb.CancelClimb())
    );

    // BALLS

    xboxDrv.y().onTrue(selector.runCoralStationCommand());
    xboxDrv.y().onFalse(selector.cancelCurrentDrivetrainCommand(CancellationSource.NA));

    // AQUA
    xboxDrv.a().onTrue(new InstantCommand(() -> selector.runAutoCommand().schedule()));
    xboxDrv.a().onFalse(selector.cancelAutoCommand());

    // Left Right
    xboxDrv.leftBumper().onTrue(new InstantCommand(() -> selector.runLeftRight(LeftRight.LEFT)));
    xboxDrv.rightBumper().onTrue(new InstantCommand(() -> selector.runLeftRight(LeftRight.RIGHT)));

    xboxDrv.leftBumper().onFalse(selector.cancelCurrentDrivetrainCommand(CancellationSource.NA).alongWith(climb.CancelClimb()));
    xboxDrv.rightBumper().onFalse(selector.cancelCurrentDrivetrainCommand(CancellationSource.NA).alongWith(climb.CancelClimb()));

    // Coral Station Run Back
    xboxDrv.button(7).onTrue(new InstantCommand(() -> selector.toggleLeftStation()).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.BALLS))));
    xboxDrv.button(8).onTrue(new InstantCommand(() -> selector.toggleRightStation()).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.BALLS))));


    //     // Manual Climb Control
    // xboxDrv.povUp().onTrue(climb.ClimbManualMode(()-> 0.4));
    // xboxDrv.povUp().onFalse(climb.CancelClimb());
    // xboxDrv.povDown().onTrue(climb.ClimbManualMode(()-> -1.0));
    // xboxDrv.povDown().onFalse(climb.CancelClimb());


    // Default driving
    Trigger escapeTrajectory = new Trigger(()->(xboxDrv.getLeftY() > 0.8));

    escapeTrajectory.onTrue(getCatzDrivetrain().cancelTrajectory());

    drive.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), drive));
    //---------------------------------------------------------------------------------------------------------------------
    // XBOX AUX
    //---------------------------------------------------------------------------------------------------------------------

    // Scoring Level Aqua determination
    xboxAux.rightTrigger().onTrue(Commands.runOnce(() -> selector.pathQueueAddBack(selector.getXBoxReefPos(), superstructure.getLevel()))
                                          .alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.AQUA))));
    // xboxAux.leftBumper().onTrue(Commands.runOnce(() -> selector.pathQueuePopFront())
    //                                     .alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.AQUA))));
    // xboxAux.rightBumper().onTrue(Commands.runOnce(() -> selector.pathQueuePopBack())
    //                                      .alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.AQUA))));
    // xboxAux.rightStick().onTrue(Commands.runOnce(() -> selector.pathQueueClear()).unless(()->xboxAux.leftStick().getAsBoolean())
    //                                     .alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.AQUA))));

    // Full Manual
    xboxAux.leftStick().and(xboxAux.rightStick()).or(isElevatorFullManual).onTrue(elevator.elevatorFullManual(() -> -xboxAux.getLeftY()).alongWith(Commands.print("Elevator Manual"))); // TODO make an override button for comp
    xboxAux.leftStick().and(xboxAux.rightStick()).or(isAlgaePivotFullManual).onTrue(algaePivot.AlgaePivotFullManualCommand(()->xboxAux.getRightY()).alongWith(Commands.print("Algae Manual")));
    isRampPivotFullManual.onTrue(rampPivot.rampPivotManual(()->xboxAux.getLeftY()).alongWith(Commands.print("Manual Ramp")));

    // Gamepiece Selection
    xboxAux.leftTrigger().onTrue(Commands.runOnce(()-> CatzSuperstructure.setChosenGamepiece(Gamepiece.CORAL)));
    xboxAux.rightTrigger().onTrue(Commands.runOnce(()-> CatzSuperstructure.setChosenGamepiece(Gamepiece.ALGAE)));

    // Scoring Action
    xboxAux.x().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.INTAKE, "container")).alongWith(Commands.print("INTAKE")));
    xboxAux.y().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, "container")).alongWith(Commands.print("OUTTAKE L" + superstructure.getLevel())));
    xboxAux.a().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.STOW, "container")).alongWith(Commands.print("STOWWW")));
    xboxAux.b().onTrue(CatzStateCommands.algaeStow(this));


    // algae punch
    DoublePressTracker.createTrigger(xboxAux.x())
                      .onTrue(algaePivot.AlgaePivot_Stow()
                                        .onlyIf(()->CatzSuperstructure.getChosenGamepiece() == Gamepiece.ALGAE)
                                        .alongWith(Commands.print("Algae Punch"))
    );

    isClimbFullManualEnabled.onTrue(Commands.print("climb full man"));

    // Scoring Level
    xboxAux.povRight().onTrue(Commands.runOnce(()-> {superstructure.setLevel(1); SmartDashboard.putNumber("Reef Level", 1);}));
    xboxAux.povUp().onTrue(Commands.runOnce(() -> {superstructure.setLevel(2); SmartDashboard.putNumber("Reef Level", 2);}));
    xboxAux.povLeft().onTrue(Commands.runOnce(() -> {superstructure.setLevel(3); SmartDashboard.putNumber("Reef Level", 3);}));
    xboxAux.povDown().onTrue(Commands.runOnce(() -> {superstructure.setLevel(4); SmartDashboard.putNumber("Reef Level", 4);}));
    //------------------------------------------------------------------------------------------------------------------------------
    //  XBOX test controls
    //------------------------------------------------------------------------------------------------------------------------------
    // xboxTest.povRight().toggleOnTrue(algaePivot.AlgaePivot_BotTop().alongWith(Commands.print("pressed POV Right"))); //TBD

    // xboxTest.povUp().toggleOnTrue(rampPivot.Ramp_Intake_Pos().alongWith(Commands.print("pressed POV Up"))); //TBD
    // xboxTest.povLeft().toggleOnTrue(rampPivot.Ramp_Climb_Pos().alongWith(Commands.print("pressed POV Left")));
    xboxTest.x().onTrue(Commands.runOnce(() -> CatzStateCommands.algaeGrndIntk(this)).alongWith(Commands.print("Hi")));
    xboxTest.a().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.STOW, "container")).alongWith(Commands.print("STOWWW")));

    xboxTest.povDown().toggleOnTrue(algaePivot.AlgaePivot_NetAlgae().alongWith(Commands.print("pressed POV Right"))); //TBD
    xboxTest.povUp().toggleOnTrue(algaePivot.AlgaePivot_Stow().alongWith(Commands.print("pressed POV Right"))); //TBD
    xboxTest.povRight().toggleOnTrue(algaeRemover.eatAlgae().alongWith(Commands.print("pressed POV Right"))); //TBD
    xboxTest.povLeft().toggleOnTrue(algaeRemover.vomitAlgae().alongWith(Commands.print("pressed POV Right"))); //TBD

    // xboxTest.x().onTrue(algaePivot.AlgaePivot_Horizontal().alongWith(Commands.print("AL:KDJF:LAKDJFLK:ADJF:LKKJAD:FLKJ")));
    // xboxTest.y().onTrue(algaePivot.AlgaePivot_Stow().alongWith(Commands.print("LAKJDFLKJALKJLKJFLSKDJLKKJSDLFKJLKKJLKKJFDKLJSLKJ")));

    // xboxTest.a().onTrue(algaeRemover.eatAlgae().alongWith(Commands.print("ea")));
    // xboxTest.b().onTrue(algaeRemover.vomitAlgae().alongWith(Commands.print("va")));

    // xboxTest.rightBumper().toggleOnTrue(algaePivot.AlgaePivot_Stow().alongWith(Commands.print("stow")));
    // xboxTest.leftBumper().toggleOnTrue(algaePivot.AlgaePivot_Horizontal().alongWith(Commands.print("eat")));
    // xboxTest.rightTrigger().onTrue(algaeRemover.eatAlgae().alongWith(Commands.print("eating algae")));
    // xboxTest.leftTrigger().onTrue(algaeRemover.vomitAlgae().alongWith(Commands.print("vomiting algae")));
    // xboxTest.rightStick().onTrue(algaeRemover.stopAlgae().alongWith(Commands.print("stop algaeing")));

    // xboxTest.a().onTrue(outtake.startIntaking().alongWith(Commands.print("INTAKEKKKEEKKEKEKEING")));
    //xboxTest.b().onTrue(outtake.startOuttake().alongWith(Commands.print("INTAKEKKKEEKKEKEKEING")));
    //xboxTest.x().onTrue(outtake.stopOuttake().alongWith(Commands.print("INTAKEKKKEEKKEKEKEING")));

    // STOWING INTAKE RAMP FOR WHATEVER REASON
    //xboxTest.start().toggleOnTrue(rampPivot.Ramp_Stow_Pos().alongWith(Commands.print("pressed start"))); //TBD
    // xboxTest.a().toggleOnTrue(elevator.Elevator_Stow().alongWith(Commands.print("L1")));
    // xboxTest.b().toggleOnTrue(elevator.Elevator_L2().alongWith(Commands.print("L2")));
    // xboxTest.x().toggleOnTrue(elevator.Elevator_L3().alongWith(Commands.print("L3")));
    // xboxTest.y().toggleOnTrue(elevator.Elevator_L4().alongWith(Commands.print("L4")));

    // xboxTest.rightStick().onTrue(elevator.elevatorFullManual(()->xboxTest.getRightY()));

    xboxTest.rightStick().onTrue(climb.ClimbManualMode(() -> xboxTest.getLeftY()*5).alongWith(Commands.print("Using manual ramp pivot")));
    xboxTest.rightStick().onFalse(climb.ClimbManualMode(()-> 0.0).alongWith(Commands.print("Nah - pivot motor")));

    // Climb
    // xboxTest.back().and(xboxTest.b()).onTrue(CatzStateCommands.climb(this).alongWith(Commands.print("climb mode"))); // Setup Climb
    // xboxTest.back().and(xboxTest.x()).onTrue(climb.fullClimb());
    // xboxTest.back().and(xboxTest.start()).onTrue(climb.ClimbManualMode(()->0.0).alongWith(Commands.print("climb manual")));
    // // Manual Climb Control
    // xboxTest.back().and(xboxTest.leftStick()).onTrue(climb.ClimbManualMode(() -> xboxTest.getLeftY()).alongWith(Commands.print("Using manual climb")));

    // Climb SetPosition Control
    // xboxTest.y().toggleOnTrue(climb.Climb_Retract().alongWith(Commands.print("pressed y")));
    // xboxTest.x().toggleOnTrue(climb.Climb_Home().alongWith(Commands.print("pressed x")));
    // xboxTest.b().toggleOnTrue(climb.Climb_Full().alongWith(Commands.print("pressed b")));

    // xboxTest.x().toggleOnTrue(rampPivot.Ramp_Stow().alongWith(Commands.print("pressed x"))); //TBD
    // xboxTest.b().toggleOnTrue(rampPivot.Ramp_Climb().alongWith(Commands.print("pressed b"))); //TBD
    // xboxTest.a().toggleOnTrue(rampPivot.Ramp_Intake().alongWith(Commands.print("pressed a")));

    // Manual Elevator Control
    Trigger leftJoystickTrigger = new Trigger(
      () -> Math.abs(xboxTest.getLeftY()) > 0.1);
    Trigger rightJoystickTrigger = new Trigger(
      () -> Math.abs(xboxTest.getRightY()) > 0.1);

    leftJoystickTrigger.onTrue(rampPivot.rampPivotManual(() -> xboxTest.getLeftY()).alongWith(Commands.print("Using manual ramp pivot")));
    leftJoystickTrigger.onFalse(rampPivot.rampPivotManual(()-> 0.0).alongWith(Commands.print("Nah - ramp motor")));

    //climb.ClimbManualMode(
    // rightJoystickTrigger.onTrue(climb.ClimbManualMode(() -> xboxTest.getRightY()).alongWith(Commands.print("Using manual climb")));
    // rightJoystickTrigger.onFalse(climb.ClimbManualMode(()-> 0.0).alongWith(Commands.print("Nah - climb motor")));

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
    );
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


  // ---------------------------------------------------------------------------
  //
  //      Subsystem getters
  //
  // ---------------------------------------------------------------------------
  public CatzDrivetrain getCatzDrivetrain() {
    return drive;
  }

  public CatzAlgaeRemover getCatzAlgaeRemover() {
    return algaeRemover;
  }

  public CatzElevator getCatzElevator() {
    return elevator;
  }

  public CatzClimb getCatzClimb() {
    return climb;
  }

  public CatzOuttake getCatzOuttake() {
    return outtake;
  }

  public Command getAutonomousCommand() {
    return auto.getCommand();
  }

  public CatzAutonomous getAutonomous(){
    return auto;
  }

  public TeleopPosSelector getSelector(){
    return selector;
  }

  public CatzAlgaePivot getAlgaePivot(){
    return algaePivot;
  }

  public CatzVision getCatzVision() {
    return vision;
  }

  public CatzRampPivot getCatzRampPivot() {
    return rampPivot;
  }

  public CatzIntakeRollers getIntakeRollers() {
    return intakeRollers;
  }

  public CatzSuperstructure getSuperstructure(){
    return superstructure;
  }
}
