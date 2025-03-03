// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.CatzAutonomous;
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
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.ControllerLEDState;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzRampPivot.CatzRampPivot;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.Alert.AlertType;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RobotContainer {

  private double SCORE_TRIGGER_THRESHHOLD = 0.8;

  // -------------------------------------------------------------------------------------------------------------------
  // Subsystem Declaration
  // -------------------------------------------------------------------------------------------------------------------
  // Primary subsystem declaration
  private static CatzDrivetrain drive = new CatzDrivetrain();

  // Assistance Subsystem declaration
  private CatzLED led = CatzLED.getInstance();
  private CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private CatzVision vision = new CatzVision(new VisionIOLimelight("limelight-tempura"),
                                             new VisionIOLimelight("limelight-gyoza"));
  private CatzOuttake outtake;
  private CatzElevator elevator = new CatzElevator();
  private CatzClimb climb = new CatzClimb();
  private CatzAlgaeRemover algaeRemover = new CatzAlgaeRemover();
  private CatzAlgaePivot algaePivot = new CatzAlgaePivot();
  private CatzRampPivot rampPivot = new CatzRampPivot();
  private CatzSuperstructure superstructure = new CatzSuperstructure(this);

  // ------------------------------------------------------------------------------------------------------------------
  // Drive Controller Declaration
  // -----------------------------------------------------------------------------------------------------------------
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);
  private CommandXboxController xboxTest = new CommandXboxController(2);
  private TeleopPosSelector selector;


  // -------------------------------------------------------------------------------------------------------------------
  // Alert Declaration
  // -------------------------------------------------------------------------------------------------------------------
  private final Alert disconnectedAlertDrive      = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert disconnectedAlertAux        = new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("Endgame Alert #2", 15.0);

  // -------------------------------------------------------------------------------------------------------------------
  // Auto Declaration
  // ---------------------------------------------------------------------------------------------------------------------
  private CatzAutonomous auto;

  public RobotContainer() {
    outtake = new CatzOuttake(this);
    auto = new CatzAutonomous(this);
    selector = new TeleopPosSelector(xboxAux, this);

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
    xboxDrv.button(8).and(xboxDrv.button(7)).onTrue(new InstantCommand(() -> {
      if(AllianceFlipUtil.shouldFlipToRed()){
        robotTracker.resetPose(new Pose2d(robotTracker.getEstimatedPose().getTranslation(), Rotation2d.k180deg));
      }else{
        robotTracker.resetPose(new Pose2d(robotTracker.getEstimatedPose().getTranslation(), new Rotation2d()));
      }
    }));

    // NBA
    xboxDrv.b().onTrue(selector.runToNearestBranch().alongWith(Commands.runOnce(()->led.setControllerState(ControllerLEDState.NBA))));
    xboxDrv.b().onFalse(selector.cancelCurrentDrivetrainCommand().alongWith(Commands.runOnce(()->led.setControllerState(ControllerLEDState.FULL_MANUAL))));

    // BALLS
    xboxDrv.y().onTrue(selector.runCoralStationCommand());
    xboxDrv.y().onFalse(selector.cancelCurrentDrivetrainCommand());

    // AQUA
    xboxDrv.a().onTrue(new InstantCommand(() -> selector.runAutoCommand().schedule()));
    xboxDrv.a().onFalse(selector.cancelAutoCommand());

    // Left Right
    xboxDrv.leftBumper().onTrue(new InstantCommand(() -> selector.runLeftRight(LeftRight.LEFT)));
    xboxDrv.rightBumper().onTrue(new InstantCommand(() -> selector.runLeftRight(LeftRight.RIGHT)));

    xboxDrv.leftTrigger().onTrue(new InstantCommand(() -> selector.runLeftRightShift(LeftRight.LEFT)));
    xboxDrv.leftTrigger().onFalse(selector.cancelCurrentDrivetrainCommand());

    xboxDrv.rightTrigger().onTrue(new InstantCommand(() -> selector.runLeftRightShift(LeftRight.RIGHT)));
    xboxDrv.rightTrigger().onFalse(selector.cancelCurrentDrivetrainCommand());
    // Score
    // xboxDrv.leftTrigger(SCORE_TRIGGER_THRESHHOLD).onTrue(new InstantCommand(() -> superstructure.setCurrentRobotAction(RobotAction.OUTTAKE)));

    // Default driving
    Trigger escapeTrajectory = new Trigger(()->(xboxDrv.getLeftY() > 0.8));
    escapeTrajectory.onTrue(selector.cancelCurrentDrivetrainCommand().alongWith(selector.cancelAutoCommand()));
    drive.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), drive));

    // Manual Climb Control
    // Trigger rightJoystickTrigger = new Trigger(
    //   () -> Math.abs(xboxTest.getRightY()) > 0.1);
    // rightJoystickTrigger.onTrue(climb.ClimbManualMode(() -> xboxTest.getRightY()).alongWith(Commands.print("Using manual climb")));

    // Manual Elevator Control
    Trigger leftJoystickTrigger = new Trigger(
      () -> Math.abs(xboxTest.getLeftY()) > 0.1);
    leftJoystickTrigger.onTrue(elevator.elevatorFullManual(() -> xboxTest.getLeftY()).alongWith(Commands.print("Using manual elevator")));
    leftJoystickTrigger.onFalse(elevator.elevatorFullManual(() -> xboxTest.getLeftY()));

    // Manual Ramp Pivot Control
    // Trigger leftJoystickTrigger = new Trigger(
    //   () -> Math.abs(xboxTest.getLeftY()) > 0.1);
    // leftJoystickTrigger.onTrue(rampPivot.rampPivotManual(() -> xboxTest.getLeftY()).alongWith(Commands.print("Using manual ramp pivot")));
    // leftJoystickTrigger.onFalse(rampPivot.rampPivotManual(()-> 0.0).alongWith(Commands.print("Nah - pivot motor")));

    // Climb SetPosition Control
    xboxTest.y().toggleOnTrue(climb.Climb_Retract().alongWith(Commands.print("pressed y")));
    xboxTest.x().toggleOnTrue(climb.Climb_Home().alongWith(Commands.print("pressed x")));
    xboxTest.b().toggleOnTrue(climb.Climb_Full().alongWith(Commands.print("pressed b")));

    xboxTest.x().toggleOnTrue(rampPivot.Ramp_Stow().alongWith(Commands.print("pressed x")));
    xboxTest.b().toggleOnTrue(rampPivot.Ramp_Climb().alongWith(Commands.print("pressed b")));

    xboxTest.rightBumper().toggleOnTrue(algaePivot.AlgaePivot_Stow().alongWith(Commands.print("stow")));
    xboxTest.leftBumper().toggleOnTrue(algaePivot.AlgaePivot_Horizontal().alongWith(Commands.print("eat")));
    xboxTest.rightTrigger().onTrue(algaeRemover.eatAlgae().alongWith(Commands.print("eating algae")));
    xboxTest.leftTrigger().onTrue(algaeRemover.vomitAlgae().alongWith(Commands.print("vomiting algae")));
    xboxTest.rightStick().onTrue(algaeRemover.stopAlgae().alongWith(Commands.print("stop algaeing")));

    // xboxTest.a().toggleOnTrue(elevator.Elevator_Stow().alongWith(Commands.print("L1")));
    // xboxTest.b().toggleOnTrue(elevator.Elevator_L2().alongWith(Commands.print("L2")));
    // xboxTest.x().toggleOnTrue(elevator.Elevator_L3().alongWith(Commands.print("L3")));
    // xboxTest.y().toggleOnTrue(elevator.Elevator_L4().alongWith(Commands.print("L4")));

    xboxTest.rightStick().onTrue(elevator.elevatorFullManual(()->xboxTest.getRightY()));

    //---------------------------------------------------------------------------------------------------------------------
    // XBOX AUX

    //---------------------------------------------------------------------------------------------------------------------
    //TODO add coral station toggle buttons

    // Scoring Level Determination
    xboxAux.rightTrigger().onTrue(Commands.runOnce(() -> selector.pathQueueAddBack(selector.getXBoxReefPos(), superstructure.getLevel())).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.AQUA))));
    xboxAux.leftBumper().onTrue(Commands.runOnce(() -> selector.pathQueuePopFront()).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.AQUA))));
    xboxAux.rightBumper().onTrue(Commands.runOnce(() -> selector.pathQueuePopBack()).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.AQUA))));
    xboxAux.rightStick().onTrue(Commands.runOnce(() -> selector.pathQueueClear()).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.AQUA))));

    xboxAux.povRight().onTrue(Commands.runOnce(()->{superstructure.setLevel(1); SmartDashboard.putNumber("Reef Level", 1);}));
    xboxAux.povUp().onTrue(Commands.runOnce(() -> {superstructure.setLevel(2); SmartDashboard.putNumber("Reef Level", 2);}));
    xboxAux.povLeft().onTrue(Commands.runOnce(() -> {superstructure.setLevel(3); SmartDashboard.putNumber("Reef Level", 3);}));
    xboxAux.povDown().onTrue(Commands.runOnce(() -> {superstructure.setLevel(4); SmartDashboard.putNumber("Reef Level", 4);}));
    // xboxAux.leftStick().onTrue(elevator.elevatorFullManual(()->xboxAux.getLeftY()));

    xboxAux.button(7).onTrue(new InstantCommand(() -> selector.toggleLeftStation()).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.BALLS))));
    xboxAux.button(8).onTrue(new InstantCommand(() -> selector.toggleRightStation()).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.BALLS))));


    // Gamepiece Selection
    xboxAux.leftTrigger().onTrue(Commands.runOnce(() -> superstructure.setChosenGamepiece(Gamepiece.CORAL)).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.FULL_MANUAL))));
    // xboxAux.rightTrigger().onTrue(Commands.runOnce(() -> superstructure.setChosenGamepiece(Gamepiece.ALGAE)).alongWith(Commands.runOnce(() -> led.setControllerState(ControllerLEDState.REMOVE_ALGAE))));

    xboxAux.y().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.OUTTAKE, "container")).alongWith(Commands.print("OUTTAKE L" + superstructure.getLevel())));
    xboxAux.x().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.INTAKE, "container")).alongWith(Commands.print("INTAKE")));
    xboxAux.b().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.INTAKE_GROUND, "container")).alongWith(Commands.print("INTAKEGROUND")));
    xboxAux.a().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.STOW, "container")).alongWith(Commands.print("STOWWW")));

    xboxAux.a().onTrue(Commands.runOnce(() -> System.out.println("L:"+superstructure.getLevel()+", "+superstructure.getChosenGamepiece())));


    xboxAux.start().onTrue(algaePivot.AlgaePivot_Punch());
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
  private Command controllerRumbleCommand() {
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

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    disconnectedAlertDrive.set(
        !DriverStation.isJoystickConnected(xboxDrv.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxDrv.getHID().getPort()));
    disconnectedAlertAux.set(
        !DriverStation.isJoystickConnected(xboxAux.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxAux.getHID().getPort()));
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

  public CatzSuperstructure getSuperstructure(){
    return superstructure;
  }
}
