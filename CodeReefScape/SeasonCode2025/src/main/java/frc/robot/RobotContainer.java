// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.Autonomous.CatzAutonomous;
import frc.robot.Autonomous.CatzAutonomous;
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

  public static boolean updateLimelight = true;

  // Subsystem Declaration
  private static CatzDrivetrain   drive                = new CatzDrivetrain();
  

  // Assistance Subsystem declaration
  private static CatzLED          led = CatzLED.getInstance();
  private static CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private static CatzVision       vision       = new CatzVision();
  // Drive Controller Declaration
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);

  // Alert Declaration
  private final Alert disconnectedAlertDrive = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert disconnectedAlertAux = new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final LoggedDashboardNumber endgameAlert1 = new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 = new LoggedDashboardNumber("Endgame Alert #2", 15.0);

  // Auto Declaration
  private AutomatedSequenceCmds autosequence = new AutomatedSequenceCmds();
  private CatzAutonomous auto = new CatzAutonomous(this);

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
    xboxAux.povRight().onTrue(ControllerModeAbstraction.sortModes(true)); // speaker
    xboxAux.povLeft().onTrue(ControllerModeAbstraction.sortModes(false)); // amp
    xboxAux.rightTrigger().onTrue(ControllerModeAbstraction.cancelController(this));    

    xboxAux.y().onTrue(ControllerModeAbstraction.robotHandoff(this)); // Handoff between shooter and intake
    xboxAux.a().onTrue(superstructure.setSuperStructureState(SuperstructureState.STOW)); // ResetPosition
    xboxAux.x().onTrue(ControllerModeAbstraction.robotScore(this, ()->xboxAux.b().getAsBoolean()));  // Score // Override
    xboxAux.back().onTrue(ControllerModeAbstraction.robotScoreSubwoofer(this, ()->xboxAux.b().getAsBoolean()));

    Trigger leftYTrigger = new Trigger(()->(xboxAux.getLeftY() > XboxInterfaceConstants.kDeadband)); // Manual ShooterPivot
    leftYTrigger.onTrue(superstructure.setShooterPivotManualPower(()->-xboxAux.getLeftY()));

    Trigger rightYTrigger = new Trigger(()->(xboxAux.getRightX() > XboxInterfaceConstants.kDeadband)); // Manual Turret
    rightYTrigger.onTrue(superstructure.setTurretManual(()->-xboxAux.getRightX()));
    
    xboxAux.rightBumper().whileTrue(rollers.setRollersIn());

    xboxAux.leftBumper().onTrue(rollers.setRollersOut().withTimeout(0.3)
                                                       .andThen(superstructure.setSuperStructureState(SuperstructureState.SCORE_AMP_PART_2)));
                                                      //  .unless(()->!superstructure.isPreviousSuperSubsystemStateScoreAmp()))
                                                      //   );
    xboxAux.leftBumper().and(xboxAux.rightBumper()).whileTrue(rollers.setRollersOff());


    /* XBOX DRIVE */
    xboxDrv.leftStick().onTrue(ControllerModeAbstraction.robotIntake(this));
    xboxDrv.rightTrigger().onTrue(ControllerModeAbstraction.cancelController(this));
    xboxDrv.start().onTrue(drive.cancelTrajectory());

    // Auto Driving
    xboxDrv.y().onTrue(new FaceTarget(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d(), drive));
    xboxDrv.b().onTrue(auto.autoFindPathAmp());
    xboxDrv.x().onTrue(auto.autoFindPathSpeaker());
    xboxDrv.a().onTrue(superstructure.setSuperStructureState(SuperstructureState.STOW));


    
    drive.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), 
                                               () -> xboxDrv.getLeftY(), 
                                               () -> xboxDrv.getRightX(), drive));
    //TODO add triggers to put default as priority    
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
    disconnectedAlertAux.setAlertOnloop(true, 1.0, 10.0);
  }

  //---------------------------------------------------------------------------
  //
  //      Subsystem getters
  //
  //---------------------------------------------------------------------------
  public CatzDrivetrain getCatzDrivetrain() {
    return drive;
  }

  public CatzSuperSubsystem getCatzSuperstructure() {
    return superstructure;
  }

  public CatzElevator getCatzElevator() {
    return elevator;
  }

  public CatzShooterFeeder getCatzShooterFeeder() {
    return shooterFeeder;
  }

  public CatzShooterFlywheels getCatzShooterFlywheels() {
    return shooterFlywheels;
  }

  public CatzIntakeRollers getCatzIntakeRollers() {
    return rollers;
  }

  public CatzIntakePivot getCatzIntakePivot() {
    return intakePivot;
  }

  public CatzAutonomous getCatzAutonomous(){
    return auto;
  }

  public Command getAutonomousCommand() {
    return auto.getCommand();
  }
}
