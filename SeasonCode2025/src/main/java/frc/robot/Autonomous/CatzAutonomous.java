// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzSubsystems.CatzStateCommands;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.*;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization.Direction;
import frc.robot.Commands.DriveAndRobotOrientationCmds.DriveToCoralStation;
import frc.robot.Commands.DriveAndRobotOrientationCmds.DriveToScore;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.RobotContainer;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.JSONUtil;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class CatzAutonomous extends SubsystemBase {
  private RobotContainer m_container;

  // ------------------------------------------------------------------------------------------------------------
  // Questionairre
  // ------------------------------------------------------------------------------------------------------------
  private static final int MAX_QUESTIONS = 5;
  private static final String AUTO_STRING = "Auto";
  private final LoggedDashboardChooser<PathPlannerAuto> autoProgramChooser = new LoggedDashboardChooser<>(AUTO_STRING + "/Program");

  private File autosDirectory            = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
  private File choreoPathsDirectory      = new File(Filesystem.getDeployDirectory(), "choreo");
  private File pathplannerPathsDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

  private JSONParser parser = new JSONParser();
  private HashMap<String, DashboardCmd> dashboardCmds = new HashMap<>();
  private PathPlannerAuto lastProgram;

  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
  private CatzDrivetrain drivetrain;

  public CatzAutonomous(RobotContainer container) {
    this.m_container = container;
    this.drivetrain = container.getCatzDrivetrain();

    // ------------------------------------------------------------------------------------------------------------
    // Autonmous questionaire gui configurations
    // ORDER MATTERS! Register named commands first, AutoBuilder second, Trajectories and add autos
    // to dashboard last
    // ------------------------------------------------------------------------------------------------------------

    // ------------------------------------------------------------------------------------------------------------
    // Path Configuration
    // ------------------------------------------------------------------------------------------------------------
    for (File pathFile : choreoPathsDirectory.listFiles()) {
      // to get rid of the extensions trailing the path names
      String pathName = pathFile.getName().replaceFirst("[.][^.]+$", "");
      try {
        NamedCommands.registerCommand(
            pathName,
            new TrajectoryDriveCmd(
                PathPlannerPath.fromChoreoTrajectory(pathName), drivetrain, true, container));
      } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
      }
    }

    NamedCommands.registerCommand("Stow", CatzStateCommands.stow(container));
    NamedCommands.registerCommand("IntakeCoralGround", CatzStateCommands.intakeCoralGround(container));
    NamedCommands.registerCommand("IntakeCoralStation", CatzStateCommands.intakeCoralStation(container));
    NamedCommands.registerCommand("IntakeAlgae", CatzStateCommands.intakeAlgae(container));
    NamedCommands.registerCommand("L1Coral", CatzStateCommands.L1Coral(container));
    NamedCommands.registerCommand("L2Coral", CatzStateCommands.L2Coral(container));
    NamedCommands.registerCommand("L3Coral", CatzStateCommands.L3Coral(container));
    NamedCommands.registerCommand("L4Coral", CatzStateCommands.L4Coral(container));
    NamedCommands.registerCommand("Processor", CatzStateCommands.processor(container));
    NamedCommands.registerCommand("BotAlgae", CatzStateCommands.intakeCoralGround(container));
    NamedCommands.registerCommand("TopAlgae", CatzStateCommands.topAlgae(container));
    NamedCommands.registerCommand("Climb", CatzStateCommands.climb(container));
    NamedCommands.registerCommand("RestPose", Commands.runOnce(()->tracker.resetPose(new Pose2d())));
    NamedCommands.registerCommand("WheelCharacterization", new WheelRadiusCharacterization(drivetrain, Direction.CLOCKWISE));

    //----------------------------------------------------------------------------------------------------
    //
    //----------------------------------------------------------------------------------------------------
    HashMap<String, Command> chooseYourOwnScoringBOT = new HashMap<>();
    chooseYourOwnScoringBOT.put("Score A", NamedCommands.getCommand("Score L"));
    chooseYourOwnScoringBOT.put("Score B", NamedCommands.getCommand("CollectGP1"));
    chooseYourOwnScoringBOT.put("Score C", NamedCommands.getCommand("CollectGP1"));
    chooseYourOwnScoringBOT.put("Score D", NamedCommands.getCommand("CollectGP1"));
    chooseYourOwnScoringBOT.put("Score E", NamedCommands.getCommand("CollectGP1"));
    chooseYourOwnScoringBOT.put("Score F", NamedCommands.getCommand("CollectGP1"));
    chooseYourOwnScoringBOT.put("Do Nothing", new PrintCommand("Skipped"));
    dashboardCmds.put("BotScoringChooser", new DashboardCmd("Score Where?", chooseYourOwnScoringBOT));

    HashMap<String, Command> chooseYourOwnScoringTOP = new HashMap<>();
    chooseYourOwnScoringTOP.put("Score A", NamedCommands.getCommand("Score A"));
    chooseYourOwnScoringTOP.put("Score H", NamedCommands.getCommand("Score H"));
    chooseYourOwnScoringTOP.put("Score I", NamedCommands.getCommand("Score I"));
    chooseYourOwnScoringTOP.put("Score J", NamedCommands.getCommand("Score J"));
    chooseYourOwnScoringTOP.put("Score K", NamedCommands.getCommand("Score K"));
    chooseYourOwnScoringTOP.put("Score L", NamedCommands.getCommand("Score L"));
    chooseYourOwnScoringTOP.put("Do Nothing", new PrintCommand("Skipped"));
    dashboardCmds.put("TopScoringChooser", new DashboardCmd("Score Where?", chooseYourOwnScoringTOP));

    HashMap<String, Command> L1orL4 = new HashMap<>();
    L1orL4.put("Score L1", NamedCommands.getCommand("L1Ccoral"));
    L1orL4.put("Score L2", NamedCommands.getCommand("L1Ccoral"));
    L1orL4.put("Score L3", NamedCommands.getCommand("L1Ccoral"));
    L1orL4.put("Score L4", NamedCommands.getCommand("L4Coral"));
    dashboardCmds.put("CoralScoringChooser", new DashboardCmd("Score Where?", L1orL4));

    for (String question : dashboardCmds.keySet()) {
      NamedCommands.registerCommand(question, dashboardCmds.get(question));
    }

    // This is not used anywhere but the library wants it
    BooleanSupplier shouldFlip = () -> AllianceFlipUtil.shouldFlipToRed();
    AutoBuilder.configure(
      tracker::getEstimatedPose,
      tracker::resetPose,
      tracker::getRobotChassisSpeeds,
      container.getCatzDrivetrain()::d,
      null,
      DriveConstants.SCORING_ROBOT_CONFIG,
      shouldFlip,
      container.getCatzDrivetrain()
    );

    // ------------------------------------------------------------------------------------------------------------
    // Path Configuration
    // ------------------------------------------------------------------------------------------------------------

    for (File autoFile : autosDirectory.listFiles()) {
      String autoName = autoFile.getName().replaceFirst("[.][^.]+$", "");
      ArrayList<Object> commands = JSONUtil.getCommandsFromAuton(autoName);
      for (Object o : commands) {
        String commandName = JSONUtil.getCommandName(o);
        System.out.println("nameeee: " + commandName);
        try {
          String[] components = commandName.split("\\+");
          Command command = null;

          if(components.length == 1){
            // Simple Trajectory Command
            command = new TrajectoryDriveCmd(PathPlannerPath.fromPathFile(commandName), drivetrain, true, container);
          } else if(components.length == 2){
            // Command Trajectory with Mechanisms
            String name = components[0];
            String action = components[1];

            if(action.equalsIgnoreCase("CS")){
              command = new DriveToCoralStation(PathPlannerPath.fromPathFile(name), m_container);
            } else if(action.contains("ReefL")){
              command = new DriveToScore(PathPlannerPath.fromPathFile(name), m_container, Integer.parseInt(action.substring("ReefL".length())));
            }
          }

          if(command == null){
            System.out.println("****** typotypotypotypotypotypotypotypotypotypotypotypotypo       \n\n\n\n\nn\n\n\n\n \n\n\n typo in pathplanner reverting to drvive forward auto ********** ");
            command = Commands.run(() -> drivetrain.drive(new ChassisSpeeds(0.5, 0.0, 0.0)), drivetrain).withTimeout(5.0);
          }
          NamedCommands.registerCommand(commandName, command);
        } catch (FileVersionException | IOException | ParseException e) {
          // e.printStackTrace();
        }
      }

      autoProgramChooser.addDefaultOption(autoName, new PathPlannerAuto(autoName));
    }
    autoProgramChooser.addOption("Wheel Characterization", new PathPlannerAuto("Wheel Characterization"));
  }


  @Override
  public void periodic() {

    // Update the list of questions
    PathPlannerAuto selectedProgram = autoProgramChooser.get();
    if (selectedProgram == null
        || selectedProgram.equals(lastProgram)
        || (DriverStation.isAutonomousEnabled() && lastProgram != null)) {
      return;
    }

    try {
      for (int i = 1; i <= MAX_QUESTIONS; i++) {
        String questionName = "Question #" + String.valueOf(i);
        SmartDashboard.putString(questionName, "");
        SmartDashboard.putData(questionName + " Response", new SendableChooser<Command>());
      }

      ArrayList<Object> commands = JSONUtil.getCommandsFromAuton(selectedProgram.getName());
      int questionCounter = 1;

      for (Object o : commands) {
        String commandName = JSONUtil.getCommandName(o);
        DashboardCmd modifiableCommand = dashboardCmds.get(commandName);

        if (modifiableCommand != null) {
          String questionName = "Question #" + String.valueOf(questionCounter);
          SmartDashboard.putString(questionName, modifiableCommand.getQuestion());
          SmartDashboard.putData(questionName + " Response", modifiableCommand.getChooser());
          questionCounter += 1;
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
    lastProgram = selectedProgram;
  }

  //----------------------------------------------------------------------------------------------------------
  //
  //          Characterization
  //
  //----------------------------------------------------------------------------------------------------------
  public Command wheelRadiusCharacterization() {
    return new WheelRadiusCharacterization(drivetrain, Direction.COUNTER_CLOCKWISE);
  }

  /** Getter for final autonomous Program */
  public Command getCommand() {

    return lastProgram;
  }

  // ---------------------------------------------------------------------------------------------------------
  //
  //          Record Types
  //
  // ---------------------------------------------------------------------------------------------------------

}
