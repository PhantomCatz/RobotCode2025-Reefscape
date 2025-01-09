// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.CatzAutonomousExternal;
import frc.robot.Autonomous.CatzAutonomousInternal;
import frc.robot.Autonomous.CatzAutonomousInternal;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.RobotSenario;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.CatzVision;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.VisionIO;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.VisionIOLimeLight;
import frc.robot.CatzSubsystems.LEDs.CatzLED;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.ControllerModeAbstraction;
import frc.robot.Commands.DriveAndRobotOrientationCmds.FaceTarget;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.Alert.AlertType;


public class RobotContainer {

  //-------------------------------------------------------------------------------------------------------------------
  // Subsystem Declaration
  //-------------------------------------------------------------------------------------------------------------------
  // Primary subsystem declaration
  private static CatzDrivetrain   drive                = new CatzDrivetrain();
  
  // Assistance Subsystem declaration
  private static CatzLED          led = CatzLED.getInstance();
  private static CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private static CatzVision       vision       = new CatzVision();

  //------------------------------------------------------------------------------------------------------------------
  // Drive Controller Declaration
  //-----------------------------------------------------------------------------------------------------------------
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);

  //-------------------------------------------------------------------------------------------------------------------
  // Alert Declaration
  //-------------------------------------------------------------------------------------------------------------------
  private final Alert disconnectedAlertDrive = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert disconnectedAlertAux = new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("Endgame Alert #2", 15.0);

  //-------------------------------------------------------------------------------------------------------------------
  // Auto Declaration
  //---------------------------------------------------------------------------------------------------------------------
  private AutomatedSequenceCmds autosequence = new AutomatedSequenceCmds();
  private CatzAutonomousInternal auto = new CatzAutonomousInternal(this);
  private CatzAutonomousExternal autoEx = new CatzAutonomousExternal(this);

  public RobotContainer() {

    // Drive And Aux Command Mapping
    configureBindings();

    // Endgame alert triggers
    new Trigger(
            () -> DriverStation.isTeleopEnabled()
                  && DriverStation.getMatchTime() > 0.0
                  && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get())
    ).onTrue(
        controllerRumbleCommand()
            .withTimeout(0.5)
            .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
            .finallyDo(() -> CatzLED.getInstance().endgameAlert = false)
    );
    new Trigger(
            () -> DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get())
    ).onTrue(
        controllerRumbleCommand()
            .withTimeout(0.2)
            .andThen(Commands.waitSeconds(0.1))
            .repeatedly()
            .withTimeout(0.9) // Rumble three times
            .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
            .finallyDo(() -> CatzLED.getInstance().endgameAlert = false)
    );
  }

  //---------------------------------------------------------------------------
  //
  //  Button mapping to commands
  //
  //---------------------------------------------------------------------------
  private void configureBindings() { // TODO organize by function
    
    /* XBOX AUX */


    /* XBOX DRIVE */
    xboxDrv.start().onTrue(drive.cancelTrajectory());

    // Auto Driving
   // xboxDrv.y().onTrue(new FaceTarget(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d(), drive));
    xboxDrv.b().onTrue(auto.autoFindPathStation());
    xboxDrv.x().onTrue(auto.autoFindPathSpeaker());


    
    drive.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), 
                                               () -> xboxDrv.getLeftY(), 
                                               () -> xboxDrv.getRightX(), drive));
    //TODO add triggers to put default as priority    
  }

  //---------------------------------------------------------------------------
  //
  //  Misc methods
  //
  //---------------------------------------------------------------------------
  /** Updates dashboard data. */ //TODO if needed add values here
  public void updateDashboardOutputs() {

  }

  /** Updates the alerts. */
  public void updateAlerts() {

  }

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
            || !DriverStation.getJoystickIsXbox(xboxDrv.getHID().getPort())
    );
    disconnectedAlertAux.set(
        !DriverStation.isJoystickConnected(xboxAux.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxAux.getHID().getPort())
    );
  }

  //---------------------------------------------------------------------------
  //
  //      Subsystem getters
  //
  //---------------------------------------------------------------------------
  public CatzDrivetrain getCatzDrivetrain() {
    return drive;
  }

  public CatzAutonomousInternal getCatzAutonomous(){
    return auto;
  }

  public Command getAutonomousCommand() {
    return autoEx.getCommand();
  }
}