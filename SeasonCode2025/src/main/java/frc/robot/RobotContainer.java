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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.RobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.driverain;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.Alert.AlertType;

public class RobotContainer {

  private final double SCORE_TRIGGER_THRESHHOLD = 0.8;

  // -------------------------------------------------------------------------------------------------------------------
  // Subsystem Declaration
  // -------------------------------------------------------------------------------------------------------------------
  // Primary subsystem declaration
  private static driverain drive = new driverain();

  // Assistance Subsystem declaration
  private RobotTracker robotTracker = RobotTracker.getInstance();


  // ------------------------------------------------------------------------------------------------------------------
  // Drive Controller Declaration
  // -----------------------------------------------------------------------------------------------------------------
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);
  private CommandXboxController xboxTest = new CommandXboxController(2);


  // -------------------------------------------------------------------------------------------------------------------
  // Alert Declaration
  // -------------------------------------------------------------------------------------------------------------------
  private final Alert disconnectedAlertDrive      = new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert disconnectedAlertAux        = new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);

  public RobotContainer() {

    // Drive And Aux Command Mapping
    configureBindings();
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

    // Default driving
    drive.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), () -> xboxDrv.getLeftY(), () -> xboxDrv.getRightX(), drive));



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
  public driverain getCatzDrivetrain() {
    return drive;
  }
}
