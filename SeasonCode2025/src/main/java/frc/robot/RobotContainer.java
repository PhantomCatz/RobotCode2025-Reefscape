// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.CatzAutonomous;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzSuperstructure.Gamepiece;
import frc.robot.CatzSubsystems.CatzSuperstructure.LeftRight;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.CatzVision;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Vision.VisionIOLimelight;
import frc.robot.CatzSubsystems.CatzElevator.*;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.Commands.AutomatedSequenceCmds;
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
  private static CatzLED led = CatzLED.getInstance();
  private static CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private static CatzVision vision = new CatzVision(new VisionIOLimelight("limelight-soba"));
  private static CatzOuttake outtake = new CatzOuttake();
  private static CatzElevator elevator = new CatzElevator();
  private static CatzSuperstructure superstructure = new CatzSuperstructure();

  // ------------------------------------------------------------------------------------------------------------------
  // Drive Controller Declaration
  // -----------------------------------------------------------------------------------------------------------------
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);

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
                .withTimeout(0.5)
                .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
                .finallyDo(() -> CatzLED.getInstance().endgameAlert = false));
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
                .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
                .finallyDo(() -> CatzLED.getInstance().endgameAlert = false));
  }

  // ---------------------------------------------------------------------------
  //
  //  Button mapping to commands
  //
  // ---------------------------------------------------------------------------

  Command currentPathfindingCommand = Commands.runOnce(() -> {});
  int POVReefAngle = 0; // 0 = none, x = x/6 revolutions
  LeftRight leftRightReef = LeftRight.LEFT;

  private void configureBindings() {
    //---------------------------------------------------------------------------------------------------------------------
    // XBOX DRIVE
    //---------------------------------------------------------------------------------------------------------------------
    xboxDrv.start().onTrue(drive.cancelTrajectory());

    // Autodrive Execution
    xboxDrv.a().onTrue(
      Commands.runOnce(
          () -> {
            currentPathfindingCommand = auto.calculateReefPos(POVReefAngle, leftRightReef);
            currentPathfindingCommand.schedule();
          }
      )
    );

    xboxDrv.a().onFalse(Commands.runOnce(() -> currentPathfindingCommand.cancel()));
    xboxDrv.a().toggleOnTrue(outtake.startIntaking().alongWith(Commands.print("pressed a")));
    xboxDrv.y().toggleOnTrue(outtake.runMotor().alongWith(Commands.print("pressed y")));

    xboxAux.a().toggleOnTrue(elevator.runMotor().alongWith(Commands.print("pressed elevator a")));
    xboxAux
        .y()
        .toggleOnTrue(elevator.runMotorBck().alongWith(Commands.print("pressed elevator y")));

    drive.setDefaultCommand(
        new TeleopDriveCmd(
            () -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), drive));


    // Auto Driving
    // Autodriving Reef Position 0-5 CCW; 0 Facing Driver Stations
    xboxDrv.povUp().onTrue(Commands.runOnce(() -> POVReefAngle = 0));
    xboxDrv.povUpLeft().onTrue(Commands.runOnce(() -> POVReefAngle = 1));
    xboxDrv.povDownLeft().onTrue(Commands.runOnce(() -> POVReefAngle = 2));
    xboxDrv.povDown().onTrue(Commands.runOnce(() -> POVReefAngle = 3));
    xboxDrv.povDownRight().onTrue(Commands.runOnce(() -> POVReefAngle = 4));
    xboxDrv.povUpRight().onTrue(Commands.runOnce(() -> POVReefAngle = 5));

    // Rung Selection
    xboxDrv.leftBumper().onTrue(Commands.runOnce(() -> leftRightReef = LeftRight.LEFT));
    xboxDrv.rightBumper().onTrue(Commands.runOnce(() -> leftRightReef = LeftRight.RIGHT));

    //---------------------------------------------------------------------------------------------------------------------
    // XBOX AUX 
    //---------------------------------------------------------------------------------------------------------------------
    xboxAux.povRight().onTrue(Commands.runOnce(()->superstructure.setLevel(1)));
    xboxAux.povUp().onTrue(Commands.runOnce(() -> superstructure.setLevel(2)));
    xboxAux.povLeft().onTrue(Commands.runOnce(() -> superstructure.setLevel(3)));
    xboxAux.povDown().onTrue(Commands.runOnce(() -> superstructure.setLevel(4)));

    xboxAux.leftBumper().onTrue(Commands.runOnce(() -> superstructure.setChosenGamepiece(Gamepiece.CORAL)));
    xboxAux.rightBumper().onTrue(Commands.runOnce(() -> superstructure.setChosenGamepiece(Gamepiece.ALGAE)));

    xboxAux.y().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.OUTTAKE)).alongWith(Commands.print("OUTTAKE")));
    xboxAux.x().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.INTAKE)).alongWith(Commands.print("INTAKE")));
    xboxAux.b().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.INTAKE_GROUND)).alongWith(Commands.print("INTAKEGROUND")));
    xboxAux.a().onTrue(Commands.runOnce(() -> superstructure.setCurrentRobotAction(RobotAction.STOW)).alongWith(Commands.print("STOWWW")));


    xboxAux.a().onTrue(Commands.runOnce(()-> System.out.println("L:"+superstructure.getLevel()+", "+superstructure.getChosenGamepiece())));
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

  public Command getAutonomousCommand() {
    return auto.getCommand();
  }

  public CatzAutonomous getAutonomous(){
    return auto;
  }
}
