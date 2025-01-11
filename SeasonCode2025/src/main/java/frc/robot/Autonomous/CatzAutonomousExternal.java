package frc.robot.Autonomous;

import java.nio.file.Path;
import java.util.Arrays;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.CharacterizationCmds.FeedForwardCharacterization;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization.Direction;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.WaitUntilPassX;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.JSONUtil;
import frc.robot.Utilities.SwitchableChooser;
import frc.robot.Utilities.VirtualSubsystem;

public class CatzAutonomousExternal extends VirtualSubsystem{
    private final int MAX_QUESTIONS = 5;
    private static final String AUTO_STRING = "Auto";

    private final LoggedDashboardChooser<PathPlannerAuto> autoProgramChooser = new LoggedDashboardChooser<>(AUTO_STRING + "/Program");
    private final List<StringPublisher> questionPublishers;
    private final List<SwitchableChooser> questionChoosers;

    private boolean trajectoriesLoaded = false;
    private HashMap<String, Command> pathplannerPaths = new HashMap<>();
    private HashMap<String, Command> choreoPaths = new HashMap<>();
    private File autosDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    private File choreoPathsDirectory = new File(Filesystem.getDeployDirectory(), "choreo");
    private File pathplannerPathsDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

    private PathPlannerAuto lastProgram;
    private List<AutoQuestionResponse> lastResponses;
    private JSONParser parser = new JSONParser();

    //---------------------------------------------------------------------------------------------------
    //  Auto Questions
    //-----------------------------------------------------------------------------------------------------
    private HashMap<String, AutoQuestion> autoQuestions = new HashMap<>();



    private RobotContainer m_container;

    public CatzAutonomousExternal(RobotContainer container) {
        this.m_container = container;
        lastResponses = List.of();

        // Publish questions and choosers
        questionPublishers = new ArrayList<>();
        questionChoosers = new ArrayList<>();
        for (int i = 0; i < MAX_QUESTIONS; i++) {
            var publisher =
                NetworkTableInstance.getDefault()
                    .getStringTopic("/SmartDashboard/" + AUTO_STRING + "/Question #" + Integer.toString(i + 1))
                    .publish();
            publisher.set("NA");
            questionPublishers.add(publisher);
            questionChoosers.add(
                new SwitchableChooser(AUTO_STRING + "/Question #" + Integer.toString(i + 1) + " Chooser"));
        }

        // Path follwing setup
        CatzRobotTracker tracker = CatzRobotTracker.getInstance();

        //------------------------------------------------------------------------------------------------------------
        // Autonmous questionaire gui configurations
        // ORDER MATTERS! Register named commands first, AutoBuilder second, Trajectories and add autos to dashboard last
        //------------------------------------------------------------------------------------------------------------

        NamedCommands.registerCommand("TestPrint", Commands.print("Bench"));

        class controller implements PathFollowingController {

            @Override
            public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose,
                    PathPlannerTrajectoryState targetState) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'calculateRobotRelativeSpeeds'");
            }

            @Override
            public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'reset'");
            }

            @Override
            public boolean isHolonomic() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'isHolonomic'");
            }
           
        }

        BooleanSupplier shouldFlip = ()-> AllianceFlipUtil.shouldFlipToRed();
        AutoBuilder.configure(
            tracker::getEstimatedPose,
            tracker::resetPose,
            tracker::getRobotChassisSpeeds,
            container.getCatzDrivetrain()::drive,
            new controller(),
            DriveConstants.TRAJECTORY_CONFIG,
            shouldFlip,
            container.getCatzDrivetrain()
        );
        
        //------------------------------------------------------------------------------------------------------------
        // Path Configuration
        //------------------------------------------------------------------------------------------------------------
        for(File pathFile : choreoPathsDirectory.listFiles()){
            //to get rid of the extensions trailing the path names
            String pathName = pathFile.getName().replaceFirst("[.][^.]+$", ""); 
            PathPlannerPath path;
            try {
                path = PathPlannerPath.fromChoreoTrajectory(pathName);
                choreoPaths.put(pathName, new TrajectoryDriveCmd(path, container.getCatzDrivetrain()));
            } catch (FileVersionException | IOException | ParseException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        NamedCommands.registerCommands(choreoPaths);
        
        for(File pathFile : pathplannerPathsDirectory.listFiles()){
            //to get rid of the extensions trailing the path names
            String pathName = pathFile.getName().replaceFirst("[.][^.]+$", ""); 
            PathPlannerPath path;
            try {
                path = PathPlannerPath.fromPathFile(pathName);
                pathplannerPaths.put(pathName, new TrajectoryDriveCmd(path, container.getCatzDrivetrain()));
            } catch (FileVersionException | IOException | ParseException e) {
                e.printStackTrace();
            }
        }

        NamedCommands.registerCommands(pathplannerPaths);
        for (File autoFile: autosDirectory.listFiles()){
            String autoName = autoFile.getName().replaceFirst("[.][^.]+$", "");
            autoProgramChooser.addOption(autoName, new PathPlannerAuto(autoName));
        }

    }

    @Override
    public void periodic() {
        // Skip updates when actively running in auto
        if (DriverStation.isAutonomousEnabled() && lastProgram != null && lastResponses == null) {
            return;
        }
        // Update the list of questions
        PathPlannerAuto selectedProgram = autoProgramChooser.get();
        if (selectedProgram == null) {
            return;
        }

        try {
            String autoName = autoProgramChooser.get().getName() + ".auto";
            JSONObject json = (JSONObject) parser.parse(new FileReader(Filesystem.getDeployDirectory()+"/pathplanner/autos/" + autoName));
            // Refresh Questionaire list when new Auto routine is selected
            if (!selectedProgram.equals(lastProgram)) {
                // Set Up Question Boxes
                for(int i = 0; i < MAX_QUESTIONS; i++) {
                    questionPublishers.get(i).set("");
                    questionChoosers.get(i).setOptions(new String[] {});
                }

                // List all auton commands in Json
                ArrayList<Object> commands = JSONUtil.getCommandsFromPath(json);
                int questionCounter = 0;

                // Fill Queston Boxes
                for(Object o : commands){
                    String commandName = JSONUtil.getCommandName(o);
                    AutoQuestion chooserAutoQuestion = autoQuestions.get(commandName);

                    if(chooserAutoQuestion != null) {
                        String questionName = "Question " + String.valueOf(questionCounter);
                        questionPublishers.get(questionCounter).set(chooserAutoQuestion.question());
                        questionChoosers.get(questionCounter).setOptions(chooserAutoQuestion.responses().stream()
                                                             .map((AutoQuestionResponse response) -> response.toString())
                                                             .toArray(String[]::new));
                        questionCounter += 1;
                    }
                }
            }   
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Update the routine and responses periodically from user
        lastProgram = selectedProgram;
    }

    //---------------------------------------------------------------------------------------------------------
    //
    //          Auto Driving
    //
    //---------------------------------------------------------------------------------------------------------
    public Command autoFindPathAmp() {
        return Commands.either(
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(1.89, 7.76, Rotation2d.fromDegrees(90)), DriveConstants.PATHFINDING_CONSTRAINTS), 
            AutoBuilder.pathfindToPose(new Pose2d(1.89, 7.76, Rotation2d.fromDegrees(90)), DriveConstants.PATHFINDING_CONSTRAINTS), 
            ()->AllianceFlipUtil.shouldFlipToRed());

    }

    public Command autoFindPathSpeaker() {
        return Commands.either(
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.74, 6.14, Rotation2d.fromDegrees(180)), DriveConstants.PATHFINDING_CONSTRAINTS), 
            AutoBuilder.pathfindToPose(new Pose2d(2.74, 6.14, Rotation2d.fromDegrees(180)), DriveConstants.PATHFINDING_CONSTRAINTS), 
            ()->AllianceFlipUtil.shouldFlipToRed());
    }
    //---------------------------------------------------------------------------------------------------------
    //
    //          Trajectory Helpers
    //
    //---------------------------------------------------------------------------------------------------------
    // private void preloadTrajectoryClass(PathPlannerPath segment) {
    //     // This is done because Java loads classes lazily. Calling this here loads the trajectory pathplanner class which
    //     // is used to follow paths and saves user code ms loop time at the start of auto.
    //     if (!trajectoriesLoaded) {
    //         trajectoriesLoaded = true;
    //         var trajectory = new PathPlannerTrajectory(
    //             segment,
    //             DriveConstants.
    //                 SWERVE_KINEMATICS.
    //                     toChassisSpeeds(CatzRobotTracker.getInstance().getCurrentModuleStates()),
    //             CatzRobotTracker.getInstance().getEstimatedPose().getRotation(),
    //             DriveConstants.TRAJECTORY_CONFIG
    //         );
    //     }
    // }

  /**
   * Resets pose accounting for alliance color.
   *
   * @param pose Pose to reset to.
   */
  public static Command resetPose(Pose2d pose) {
    return Commands.runOnce(
        () -> {
          CatzRobotTracker.getInstance().resetPose(AllianceFlipUtil.apply(pose));
          CatzRobotTracker.getInstance().addTrajectorySetpointData(AllianceFlipUtil.apply(pose));
        });
  }





    /** Getter for final autonomous Program */
    public Command getCommand() { 
        return lastProgram;
    }


    //---------------------------------------------------------------------------------------------------------
    //
    //          Record Types
    //
    //---------------------------------------------------------------------------------------------------------
    /** A customizable auto routine associated with a single command. */
    private static final record DashboardAutoProgram(
        String name, List<AutoQuestion> questions, Command command) {}

    /** A question to ask for customizing an auto routine. */
    public static record AutoQuestion(String question, List<AutoQuestionResponse> responses) {}

    /** Responses to auto routine questions. */
    public static enum AutoQuestionResponse {
        AMP,
        CENTER,
        SOURCE,
        ONE,
        TWO,
        THREE,
        SOURCE_WALL,
        SOURCE_MIDDLE,
        MIDDLE,
        AMP_MIDDLE,
        AMP_WALL,
        SCORE_POOPED,
        FOURTH_CENTER,
        THINKING_ON_YOUR_FEET,
        IMMEDIATELY,
        SIX_SECONDS,
        LAST_SECOND,
        YES,
        NO
    }


}