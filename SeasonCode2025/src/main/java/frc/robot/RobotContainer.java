// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.DepreciatedVision.VisionConstants.SOBA_TRANSFORM;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.CatzAutonomousExternal;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.RobotSenario;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.DepreciatedVision.VisionIOLimeLight;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Vision.CatzVision;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Vision.VisionIO;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Vision.VisionIOPhotonVisionSim;
import frc.robot.CatzSubsystems.LEDs.CatzLED;
import frc.robot.CatzSubsystems.Outtake.CatzOuttake;
import frc.robot.CatzSubsystems.Outtake.OuttakeIO;
import frc.robot.CatzSubsystems.Outtake.OuttakeIOSparkmax;
import frc.robot.CatzSubsystems.Outtake.CatzOuttake.outtakeStates;
import frc.robot.CatzSubsystems.Elevator.*;
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
  private static CatzVision       vision = new CatzVision(new VisionIOPhotonVisionSim("SOBA", SOBA_TRANSFORM, () -> robotTracker.getEstimatedPose()));

  private static CatzOuttake      outtake = new CatzOuttake();
  private static CatzElevator     elevator = new CatzElevator();

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
  Command pathfindToOrigin = Commands.runOnce(()->{});
  private void configureBindings() { // TODO organize by function
    
    /* XBOX AUX */


    /* XBOX DRIVE */
    xboxDrv.start().onTrue(drive.cancelTrajectory());

    // Auto Driving
   // xboxDrv.y().onTrue(new FaceTarget(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d(), drive));
    xboxDrv.b().toggleOnTrue(Commands.startEnd(
      () -> {
        pathfindToOrigin = autoEx.getPathfindingCommand(new Pose2d(2, 7, new Rotation2d()));
        pathfindToOrigin.schedule();
        System.out.println("scheduled");
    }, 
      () -> {
        pathfindToOrigin.cancel();
        System.out.println("Canceled");
    }));

    xboxDrv.a().toggleOnTrue(outtake.startIntaking().alongWith(Commands.print("pressed a")));
    xboxDrv.y().toggleOnTrue(outtake.runMotor().alongWith(Commands.print("pressed y")));

    xboxAux.a().toggleOnTrue(elevator.runMotor().alongWith(Commands.print("pressed elevator a")));
    xboxAux.y().toggleOnTrue(elevator.runMotorBck().alongWith(Commands.print("pressed elevator y")));
    
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

  public Command getAutonomousCommand() {
    return autoEx.getCommand();
  }
}
