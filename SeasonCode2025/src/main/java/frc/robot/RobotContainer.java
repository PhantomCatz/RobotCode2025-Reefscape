// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.DepreciatedVision.VisionConstants.SOBA_TRANSFORM;

import java.util.HashMap;

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
import frc.robot.Autonomous.CatzAutonomous;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.RobotSenario;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.DepreciatedVision.VisionIOLimeLight;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Vision.CatzVision;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Vision.VisionIO;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Vision.VisionIOPhotonVisionSim;
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
  private static CatzVision       vision = new CatzVision(new VisionIOPhotonVisionSim("SOBA", SOBA_TRANSFORM, () -> robotTracker.getEstimatedPose()));

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
  Command currentPathfindingCommand = Commands.runOnce(()->{});
  HashMap<Rotation2d, Boolean> reefAngleToggles = new HashMap<>();
  int bumperLeftRight = 0; // 0 = none, 1 = left, -1 = right

  private void configureBindings() {
    /* XBOX AUX */

    /* XBOX DRIVE */
    xboxDrv.start().onTrue(drive.cancelTrajectory());

    // Auto Driving
    //   xboxDrv.y().onTrue(new FaceTarget(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d(), drive));
    //   xboxDrv.b().toggleOnTrue(Commands.runOnce(() -> {
    //       pathfindToOrigin = auto.getPathfindingCommand(new Pose2d(2, 7, new Rotation2d()));
    //       pathfindToOrigin.schedule();
    //   }));

    //   xboxDrv.b().toggleOnFalse(Commands.runOnce(() -> {
    //     pathfindToOrigin.cancel();
    //   }));

    xboxDrv.povUp().onTrue(Commands.runOnce(() ->        reefAngleToggles.put(Rotation2d.fromRotations(0.0/6.0), true)));
    xboxDrv.povUpLeft().onTrue(Commands.runOnce(() ->    reefAngleToggles.put(Rotation2d.fromRotations(1.0/6.0), true)));
    xboxDrv.povDownLeft().onTrue(Commands.runOnce(() ->  reefAngleToggles.put(Rotation2d.fromRotations(2.0/6.0), true)));
    xboxDrv.povDown().onTrue(Commands.runOnce(() ->      reefAngleToggles.put(Rotation2d.fromRotations(3.0/6.0), true)));
    xboxDrv.povDownRight().onTrue(Commands.runOnce(() -> reefAngleToggles.put(Rotation2d.fromRotations(4.0/6.0), true)));
    xboxDrv.povUpRight().onTrue(Commands.runOnce(() ->   reefAngleToggles.put(Rotation2d.fromRotations(5.0/6.0), true)));

    xboxDrv.povUp().onFalse(Commands.runOnce(() ->        reefAngleToggles.put(Rotation2d.fromRotations(0.0/6.0), false)));
    xboxDrv.povUpLeft().onFalse(Commands.runOnce(() ->    reefAngleToggles.put(Rotation2d.fromRotations(1.0/6.0), false)));
    xboxDrv.povDownLeft().onFalse(Commands.runOnce(() ->  reefAngleToggles.put(Rotation2d.fromRotations(2.0/6.0), false)));
    xboxDrv.povDown().onFalse(Commands.runOnce(() ->      reefAngleToggles.put(Rotation2d.fromRotations(3.0/6.0), false)));
    xboxDrv.povDownRight().onFalse(Commands.runOnce(() -> reefAngleToggles.put(Rotation2d.fromRotations(4.0/6.0), false)));
    xboxDrv.povUpRight().onFalse(Commands.runOnce(() ->   reefAngleToggles.put(Rotation2d.fromRotations(5.0/6.0), false)));

    xboxDrv.leftBumper().onTrue(Commands.runOnce(() -> bumperLeftRight = 1));
    xboxDrv.leftBumper().onFalse(Commands.runOnce(() -> bumperLeftRight = 0));
    xboxDrv.rightBumper().onTrue(Commands.runOnce(() -> bumperLeftRight = -1));
    xboxDrv.rightBumper().onFalse(Commands.runOnce(() -> bumperLeftRight = 0));

    xboxDrv.a().onTrue(Commands.runOnce(() -> {
      Rotation2d selectedAngle = new Rotation2d();
      for(Rotation2d angle: reefAngleToggles.keySet()){
        if(reefAngleToggles.get(angle)){
          selectedAngle = angle;
        }
      }

      Translation2d unitRadius = new Translation2d(selectedAngle.getCos(), selectedAngle.getSin());
      Translation2d unitLeftRight = unitRadius.rotateBy(Rotation2d.fromDegrees(90));
      Translation2d radius = unitRadius.times(Reef.reefOrthogonalRadius + Reef.scoringDistance);
      Translation2d leftRight = unitLeftRight.times(bumperLeftRight * Reef.leftRightDistance);
      if (unitLeftRight.getY() < 0){
        leftRight = leftRight.times(-1);
      }

      Translation2d scoringPos = radius.plus(leftRight).plus(Reef.center);
      currentPathfindingCommand = auto.getPathfindingCommand(new Pose2d(scoringPos, selectedAngle));
      currentPathfindingCommand.schedule();
    }));

    xboxDrv.a().onFalse(Commands.runOnce(() -> currentPathfindingCommand.cancel()));
    
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
    return auto.getCommand();
  }
}
