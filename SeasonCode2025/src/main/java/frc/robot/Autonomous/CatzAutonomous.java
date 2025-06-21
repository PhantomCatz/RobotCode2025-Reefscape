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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.*;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization.Direction;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.RobotContainer;
import frc.robot.TeleopPosSelector;
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
  private static CatzAutonomous instance;

  public static CatzAutonomous getInstance(){
    if(instance == null) instance = new CatzAutonomous();
    return instance;
  }

  // ------------------------------------------------------------------------------------------------------------
  // Questionairre
  // ------------------------------------------------------------------------------------------------------------
  private static final int MAX_QUESTIONS = 5;
  private static final String AUTO_STRING = "Auto";
  private final LoggedDashboardChooser<PathPlannerAuto> autoProgramChooser = new LoggedDashboardChooser<>(AUTO_STRING + "/Program");
  private final SendableChooser<Boolean> waitForCoralChooser = new SendableChooser<>();

  private File autosDirectory            = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
  private File choreoPathsDirectory      = new File(Filesystem.getDeployDirectory(), "choreo");
  private File pathplannerPathsDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

  private JSONParser parser = new JSONParser();
  private HashMap<String, DashboardCmd> dashboardCmds = new HashMap<>();
  private PathPlannerAuto lastProgram;

  private final CatzRobotTracker tracker = CatzRobotTracker.getInstance();
  private final CatzDrivetrain drivetrain = CatzDrivetrain.getInstance();
  private final RobotContainer container = RobotContainer.getInstance();
  private final CatzSuperstructure superstructure = CatzSuperstructure.getInstance();

  private CatzAutonomous() {

    SmartDashboard.putData("Wait For Coral?", waitForCoralChooser);

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
                PathPlannerPath.fromChoreoTrajectory(pathName), true, false));
      } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
      }
    }
    TeleopPosSelector selector = TeleopPosSelector.getInstance();

    NamedCommands.registerCommand("Stow", superstructure.stow());
    NamedCommands.registerCommand("IntakeCoralStation", superstructure.intakeCoralStation());
    NamedCommands.registerCommand("IntakeAlgae", superstructure.intakeAlgae());
    NamedCommands.registerCommand("L1Coral", superstructure.L1Coral());
    NamedCommands.registerCommand("L2Coral", superstructure.L2Coral());
    NamedCommands.registerCommand("L3Coral", superstructure.L3Coral());
    NamedCommands.registerCommand("L4Coral", superstructure.L4Coral());
    NamedCommands.registerCommand("BotAlgae", superstructure.botAlgae());
    NamedCommands.registerCommand("TopAlgae", superstructure.topAlgae());
    NamedCommands.registerCommand("RestPose", Commands.runOnce(()->tracker.resetPose(new Pose2d())));
    NamedCommands.registerCommand("WheelCharacterization", new WheelRadiusCharacterization(drivetrain, Direction.CLOCKWISE));

    NamedCommands.registerCommand("AlgaeTop", superstructure.topAlgae());
    NamedCommands.registerCommand("AlgaeBot", superstructure.botAlgae());

    HashMap<String, Command> startPose = new HashMap<>();
    HashMap<String, Command> reefPose1 = new HashMap<>();
    HashMap<String, Command> reefPose2 = new HashMap<>();
    HashMap<String, Command> reefPose3 = new HashMap<>();
    HashMap<String, Command> reefPose4 = new HashMap<>();

    waitForCoralChooser.setDefaultOption("No", false);
    // waitForCoralChooser.("Yes", true);
    waitForCoralChooser.addOption("Yes", true);

    // for(int i = 0; i < FieldConstants.StartPoses.startArray.length; i++){
    //   final int j = i;
    //   Supplier<Pose2d> startPoseSupplier = ()->AllianceFlipUtil.apply(new Pose2d(FieldConstants.StartPoses.startArray[j], Rotation2d.k180deg));
    //   startPose.put(""+(i+1), new InstantCommand(()->CatzRobotTracker.getInstance().resetPose(startPoseSupplier)));
    // }
    // dashboardCmds.put("StartChooser", new DashboardCmd("Start Where?", startPose));

    // for(int side = 0; side < 6; side++){
    //   for(LeftRight leftRight : LeftRight.values()){
    //     String letter = selector.getPoseToLetter(""+side + " " + leftRight);
    //     final int s = side;

    //     NamedCommands.registerCommand(letter, new SeqCmd(
    //         superstructure.driveToScore(() ->selector.getPathfindingPath(()->selector.calculateReefPose(s, leftRight, false)), 4),
    //         superstructure.driveToCoralStation(()->selector.getPathfindingPath(()->selector.getBestCoralStation()), () -> waitForCoralChooser.getSelected())
    //       )
    //     );

    //     reefPose1.put(letter, NamedCommands.getCommand(letter));
    //     reefPose2.put(letter, NamedCommands.getCommand(letter));
    //     reefPose3.put(letter, NamedCommands.getCommand(letter));
    //     reefPose4.put(letter, NamedCommands.getCommand(letter));
    //   }
    // }

    // reefPose1.put("None", new InstantCommand());
    // reefPose2.put("None", new InstantCommand());
    // reefPose3.put("None", new InstantCommand());
    // reefPose4.put("None", new InstantCommand());

    // dashboardCmds.put("ReefChooser1", new DashboardCmd("First Pose?", reefPose1));
    // dashboardCmds.put("ReefChooser2", new DashboardCmd("Second pose?", reefPose2));
    // dashboardCmds.put("ReefChooser3", new DashboardCmd("Third Pose?", reefPose3));
    // dashboardCmds.put("ReefChooser4", new DashboardCmd("Fourth Pose?", reefPose4));
    //----------------------------------------------------------------------------------------------------
    //
    //----------------------------------------------------------------------------------------------------

    for (String question : dashboardCmds.keySet()) {
      NamedCommands.registerCommand(question, dashboardCmds.get(question));
    }

    BooleanSupplier shouldFlip = () -> AllianceFlipUtil.shouldFlipToRed();

    // This is not used anywhere but the library wants it
    AutoBuilder.configure(
      tracker::getEstimatedPose,
      tracker::resetPose,
      tracker::getRobotChassisSpeeds,
      drivetrain::d,
      null,
      DriveConstants.TRAJ_ROBOT_CONFIG,
      shouldFlip,
      drivetrain
    );

    // ------------------------------------------------------------------------------------------------------------
    // Path Configuration
    // ------------------------------------------------------------------------------------------------------------

    for (File autoFile : autosDirectory.listFiles()) {
      String autoName = autoFile.getName().replaceFirst("[.][^.]+$", "");
      ArrayList<Object> commands = JSONUtil.getCommandsFromAuton(autoName);
        for (Object o : commands) {
          String commandName = JSONUtil.getCommandName(o);
          if(commandName == null) {
              System.out.println("commandName Null for " + autoName);
              continue;
          }
          try {
            String[] components = commandName.split("\\+");
            Command command = null;

            if(components.length == 1) {
              String action = components[0];

              if(action.contains("wait")) {
                String waitTime = action.replace("wait", "");
                waitTime.strip();
                command = Commands.waitSeconds(Double.parseDouble(waitTime));
              }
              else {
                // Simple Trajectory Command
                command = new TrajectoryDriveCmd(PathPlannerPath.fromPathFile(commandName), false, false);
              }
            } else if(components.length >= 2){
              // Command Trajectory with Mechanisms
              String name = components[0];
              String action = components[1];

              PathPlannerPath path = PathPlannerPath.fromPathFile(name);

              if(action.equalsIgnoreCase("CS")) {
                command = superstructure.driveToCoralStation(path, () -> waitForCoralChooser.getSelected());
              } else if(action.contains("ReefL")) {
                if(components.length == 3){
                  String d = components[2];
                  double delay = Double.parseDouble(d.substring("d".length()));
                  command = superstructure.driveToScore(path, Integer.parseInt(action.substring("ReefL".length())), delay);
                }else{
                  command = superstructure.driveToScore(path, Integer.parseInt(action.substring("ReefL".length())));
                }
              }


            }

            if(command == null){
              System.out.println("****** typotypotypotypotypotypotypotypotypotypotypotypotypo       \n\n\n\n\nn\n\n\n\n \n\n\n typo in pathplanner reverting to drvive forward auto ********** ");
              command = Commands.run(() -> drivetrain.drive(new ChassisSpeeds(0.5, 0.0, 0.0)), drivetrain).withTimeout(5.0); //TODO this only drives to the right(field relative). so make this different for blue alliance
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

  PathPlannerAuto selectedProgram;
  @Override
  public void periodic() {

    // Update the list of questions
    selectedProgram = autoProgramChooser.get();
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
    System.out.println("program name: " + selectedProgram.getName());
    return selectedProgram;
  }

  // ---------------------------------------------------------------------------------------------------------
  //
  //          Record Types
  //
  // ---------------------------------------------------------------------------------------------------------

}
