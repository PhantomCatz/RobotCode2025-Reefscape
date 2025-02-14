// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

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
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RobotContainer {

  // -------------------------------------------------------------------------------------------------------------------
  // Subsystem Declaration
  // -------------------------------------------------------------------------------------------------------------------
  // Primary subsystem declaration
  private static CatzDrivetrain drive = new CatzDrivetrain();

  // Assistance Subsystem declaration
  private CatzLED led = CatzLED.getInstance();
  private CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private CatzVision vision = new CatzVision(new VisionIOLimelight("limelight-soba"));
  private CatzOuttake outtake = new CatzOuttake();
  private CatzElevator elevator = new CatzElevator();
  private CatzClimb climb = new CatzClimb();
  private CatzAlgaeRemover algaeRemover = new CatzAlgaeRemover();
  private CatzAlgaePivot algaePivot = new CatzAlgaePivot();
  private CatzSuperstructure superstructure = new CatzSuperstructure(this);

  // ------------------------------------------------------------------------------------------------------------------
  // Drive Controller Declaration
  // -----------------------------------------------------------------------------------------------------------------
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);
  private CommandXboxController xboxTest = new CommandXboxController(2);

  private TeleopPosSelector selector = new TeleopPosSelector(xboxAux, this);

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
  private CatzAutonomous auto = new CatzAutonomous(this);

  public RobotContainer() {
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

  private int POVReefAngle = 0;

  private void configureBindings() {
    //---------------------------------------------------------------------------------------------------------------------
    // XBOX Drive
    //---------------------------------------------------------------------------------------------------------------------
    // Reef autopathfind
    xboxDrv.b().onTrue(selector.runReefPathfindingCommand(() -> selector.getClosestReefPos()));
    xboxDrv.b().onFalse(selector.stopPathfindingCommand());

    xboxDrv.a().onTrue(selector.runReefPathfindingCommand(() -> selector.pathQueuePeekFront().getFirst()).alongWith(new InstantCommand(() -> selector.pathQueuePopFront())));
    xboxDrv.a().onFalse(selector.stopPathfindingCommand());

    xboxDrv.leftBumper().onTrue(selector.runLeftRightCommand(LeftRight.LEFT));
    xboxDrv.leftBumper().onFalse(selector.stopPathfindingCommand());

    xboxDrv.leftBumper().and(xboxDrv.rightBumper()).onTrue(selector.stopPathfindingCommand());

    xboxDrv.rightBumper().onTrue(selector.runLeftRightCommand(LeftRight.RIGHT));
    xboxDrv.rightBumper().onFalse(selector.stopPathfindingCommand());

    xboxDrv.rightTrigger().onTrue(selector.runCoralStationPathFindingCommand(()-> FieldConstants.CoralStation.rightCenterFace));
    xboxDrv.rightTrigger().onFalse(selector.stopPathfindingCommand());

    xboxDrv.leftTrigger().onTrue(selector.runCoralStationPathFindingCommand(()-> FieldConstants.CoralStation.leftCenterFace));
    xboxDrv.leftTrigger().onFalse(selector.stopPathfindingCommand());

    xboxDrv.rightTrigger().and(xboxDrv.leftTrigger()).onTrue(selector.stopPathfindingCommand());

    // Default driving
    drive.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), drive));

    // Manual Climb Control
    Trigger rightJoystickTrigger = new Trigger(
      () -> Math.abs(xboxTest.getRightY()) > 0.1);
    rightJoystickTrigger.onTrue(climb.ClimbManualMode(() -> xboxTest.getRightY()).alongWith(Commands.print("Using manual climb")));

    // Manual Elevator Control
    Trigger leftJoystickTrigger = new Trigger(
      () -> Math.abs(xboxTest.getLeftY()) > 0.1);
    leftJoystickTrigger.onTrue(elevator.elevatorFullManual(() -> xboxTest.getLeftY()).alongWith(Commands.print("Using manual elevator")));

    // Climb SetPosition Control
    xboxDrv.y().toggleOnTrue(climb.Climb_Retract().alongWith(Commands.print("pressed y")));
    xboxDrv.x().toggleOnTrue(climb.Climb_Home().alongWith(Commands.print("pressed x")));
    xboxDrv.b().toggleOnTrue(climb.Climb_Full().alongWith(Commands.print("pressed b")));

    xboxTest.rightBumper().toggleOnTrue(algaePivot.AlgaePivot_Stow().alongWith(Commands.print("stow")));
    xboxTest.leftBumper().toggleOnTrue(algaePivot.AlgaePivot_Horizontal().alongWith(Commands.print("stow")));

    xboxTest.a().toggleOnTrue(elevator.Elevator_L1().alongWith(Commands.print("L1")));
    xboxTest.b().toggleOnTrue(elevator.Elevator_L2().alongWith(Commands.print("L2")));
    xboxTest.x().toggleOnTrue(elevator.Elevator_L3().alongWith(Commands.print("L3")));
    xboxTest.y().toggleOnTrue(elevator.Elevator_L4().alongWith(Commands.print("L4")));

    xboxTest.leftTrigger().onTrue(outtake.startIntaking().alongWith(Commands.print("intake")));
    xboxTest.rightTrigger().onTrue(outtake.startOuttake().alongWith(Commands.print("Outtaking")));
    xboxTest.leftBumper().onTrue(outtake.outtakeL4().alongWith(Commands.print("Outtaking L4")));

    xboxTest.rightStick().onTrue(elevator.elevatorFullManual(()->xboxTest.getRightY()));

    //---------------------------------------------------------------------------------------------------------------------
    // XBOX AUX
    //---------------------------------------------------------------------------------------------------------------------
    // Reef autopathfind

    // Scoring Level Determination
    xboxAux.rightTrigger().onTrue(Commands.runOnce(() -> selector.pathQueueAddBack(selector.getXBoxReefPos(), superstructure.getLevel())));
    xboxAux.y().onTrue(Commands.runOnce(() -> selector.pathQueuePopFront()));
    xboxAux.b().onTrue(Commands.runOnce(() -> selector.pathQueuePopBack()));
    xboxAux.rightStick().onTrue(Commands.runOnce(() -> selector.pathQueueClear()));

    xboxAux.povRight().onTrue(Commands.runOnce(()->{superstructure.setLevel(1); SmartDashboard.putNumber("Reef Level", 1);}));
    xboxAux.povUp().onTrue(Commands.runOnce(() -> {superstructure.setLevel(2); SmartDashboard.putNumber("Reef Level", 2);}));
    xboxAux.povLeft().onTrue(Commands.runOnce(() -> {superstructure.setLevel(3); SmartDashboard.putNumber("Reef Level", 3);}));
    xboxAux.povDown().onTrue(Commands.runOnce(() -> {superstructure.setLevel(4); SmartDashboard.putNumber("Reef Level", 4);}));

    // Gamepiece Selection
    xboxAux.leftBumper().onTrue(Commands.runOnce(() -> superstructure.setChosenGamepiece(Gamepiece.CORAL)));
    xboxAux.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setChosenGamepiece(Gamepiece.ALGAE)));

    xboxAux.y().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.OUTTAKE)).alongWith(Commands.print("OUTTAKE L" + superstructure.getLevel())));
    xboxAux.x().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.INTAKE)).alongWith(Commands.print("INTAKE")));
    xboxAux.b().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.INTAKE_GROUND)).alongWith(Commands.print("INTAKEGROUND")));
    xboxAux.a().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.STOW)).alongWith(Commands.print("STOWWW")));

    xboxAux.a().onTrue(Commands.runOnce(() -> System.out.println("L:"+superstructure.getLevel()+", "+superstructure.getChosenGamepiece())));
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

  public int getReefAngle(){
    return POVReefAngle;
  }

}
