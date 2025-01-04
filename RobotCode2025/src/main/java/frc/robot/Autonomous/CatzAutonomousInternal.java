package frc.robot.Autonomous;

import java.nio.file.Path;
import java.util.Arrays;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.DRIVE_CONFIG;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.TRAJECTORY_CONFIG;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.getNewHolController;
import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.getNewPathFollowingController;

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
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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

/*********************************************************************************
 * 
 *   Autonomous class housing all autonomous routines and related functions
 * 
 *********************************************************************************/
public class CatzAutonomousInternal extends VirtualSubsystem{
    private final int MAX_QUESTIONS = 5;
    private static final AutoProgram defaultRoutine =
                new AutoProgram("Do Nothing", List.of(), Commands.none());
    private static final String AUTO_STRING = "Auto";

    private final LoggedDashboardChooser<AutoProgram> autoProgramChooser = new LoggedDashboardChooser<>(AUTO_STRING + "/Routine");
    private final List<StringPublisher> questionPublishers;
    private final List<SwitchableChooser> questionChoosers;

    private boolean trajectoriesLoaded = false;
    private HashMap<String, Command> pathplannerPaths = new HashMap<>();
    private HashMap<String, Command> choreoPaths = new HashMap<>();
    private File choreoPathsDirectory = new File(Filesystem.getDeployDirectory(), "choreo");
    private File pathplannerPathsDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

    private AutoProgram lastRoutine;
    private List<AutoQuestionResponse> lastResponses;

    private RobotContainer m_container;

    public CatzAutonomousInternal(RobotContainer container) {
        this.m_container = container;
        autoProgramChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
        lastRoutine = defaultRoutine;
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

        BooleanSupplier shouldFlip = () -> AllianceFlipUtil.shouldFlipToRed();

        AutoBuilder.configure(
            tracker::getEstimatedPose,
            tracker::resetPose,
            tracker::getRobotChassisSpeeds,
            container.getCatzDrivetrain()::drive,
            getNewPathFollowingController(),
            TRAJECTORY_CONFIG,
            shouldFlip,
            container.getCatzDrivetrain()
        );
        
        //------------------------------------------------------------------------------------------------------------
        // Path Configuration
        //------------------------------------------------------------------------------------------------------------ //TODO issues with string conversions
        // for(File pathFile : choreoPathsDirectory.listFiles()){
        //     //to get rid of the extensions trailing the path names
        //     String pathName = pathFile.getName().replaceFirst("[.][^.]+$", "");
        //     PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
        //     choreoPaths.put(pathName, new TrajectoryDriveCmd(path, container.getCatzDrivetrain()));
        // }
        // NamedCommands.registerCommands(choreoPaths);

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
        
        addProgram("Speaker",  
            List.of(
                new AutoQuestion(
                    "Starting location?",
                    List.of(
                        AutoQuestionResponse.AMP,
                        AutoQuestionResponse.CENTER
                    )
                ),
                new AutoQuestion(
                    "How many spike notes?",
                    List.of(
                            AutoQuestionResponse.TWO,
                            AutoQuestionResponse.THREE
                    )
                )
                    ), 
            speakerSideAuto()
        );
    } //end of CatzAutonomous constructor

    @Override
    public void periodic() {
        // Skip updates when actively running in auto
        if (DriverStation.isAutonomousEnabled() && lastRoutine != null && lastResponses == null) {
            return;
        }
        // Update the list of questions
        var selectedRoutine = autoProgramChooser.get();
        if (selectedRoutine == null) {
            return;
        }

        // Refresh Questionaire list when new Auto routine is selected
        if (!selectedRoutine.equals(lastRoutine)) 
        {
            List<AutoQuestion> questions = selectedRoutine.questions();
            for (int i = 0; i < MAX_QUESTIONS; i++) 
            {
                if (i < questions.size()) 
                {
                    questionPublishers.get(i).set(questions.get(i).question());
                    questionChoosers.get(i).setOptions(questions.get(i).responses().stream()
                                .map((AutoQuestionResponse response) -> response.toString())
                                .toArray(String[]::new)
                    );
                } else 
                {
                    questionPublishers.get(i).set("");
                    questionChoosers.get(i).setOptions(new String[] {});
                }
            }
        }

        // Update the routine and responses periodically from user
        lastRoutine = selectedRoutine;
        lastResponses = new ArrayList<>();
        for (int i = 0; i < lastRoutine.questions().size(); i++) {
            String responseString = questionChoosers.get(i).get();
            lastResponses.add(
                responseString == null
                    ? lastRoutine.questions().get(i).responses().get(0)
                    : AutoQuestionResponse.valueOf(responseString)
            );
        }

    }//end of periodic()

    //---------------------------------------------------------------------------------------------------------
    //
    //          Chooser helpers
    //
    //---------------------------------------------------------------------------------------------------------
    /** Registers a new auto routine that can be selected. */
    private void addProgram(String name, Command command) {
        addProgram(name, List.of(), command);
    }

    /** Registers a new auto routine that can be selected. */
    private void addProgram(String name, List<AutoQuestion> questions, Command command) {
        if (questions.size() > MAX_QUESTIONS) {
        throw new RuntimeException(
            "Auto routine contained more than "
                + Integer.toString(MAX_QUESTIONS)
                + " questions: "
                + name);
        }
        autoProgramChooser.addOption(name, new AutoProgram(name, questions, command));
    }

    //---------------------------------------------------------------------------------------------------------
    //
    //          Autonomous Paths
    //
    //---------------------------------------------------------------------------------------------------------
    public Command speakerSideAuto() {
        HashMap<AutoQuestionResponse, Command> startingChoices = new HashMap<>();
        startingChoices.put(AutoQuestionResponse.AMP, NamedCommands.getCommand("Example Path"));
        startingChoices.put(AutoQuestionResponse.CENTER, NamedCommands.getCommand("Wing Scoring 2"));



        return Commands.sequence(
            Commands.select(
                Map.of(
                    AutoQuestionResponse.SOURCE,
                    resetPose(new Pose2d()),
                    AutoQuestionResponse.CENTER,
                    resetPose(new Pose2d()),
                    AutoQuestionResponse.AMP,
                    resetPose(new Pose2d(2,5, Rotation2d.fromDegrees(0)))
                ),
                () -> lastResponses.get(0) // Starting location
            ),
            Commands.select(startingChoices, () -> lastResponses.get(0)));
    }


    //---------------------------------------------------------------------------------------------------------
    //
    //          Characterization Routines
    //
    //---------------------------------------------------------------------------------------------------------

    
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
    private void preloadTrajectoryClass(PathPlannerPath segment) {
        // This is done because Java loads classes lazily. Calling this here loads the trajectory pathplanner class which
        // is used to follow paths and saves user code ms loop time at the start of auto.
        if (!trajectoriesLoaded) {
            trajectoriesLoaded = true;
            var trajectory = new PathPlannerTrajectory(
                segment,
                DriveConstants.
                    SWERVE_KINEMATICS.
                        toChassisSpeeds(CatzRobotTracker.getInstance().getCurrentModuleStates()),
                CatzRobotTracker.getInstance().getEstimatedPose().getRotation(),
                DriveConstants.TRAJECTORY_CONFIG
            );
        }
    }

  /**
   * Resets pose accounting for alliance color.
   *
   * @param pose Pose to reset to.
   */
  public static Command resetPose(Pose2d pose) {
    return Commands.runOnce(
        () -> {
          CatzRobotTracker.getInstance().resetPose(AllianceFlipUtil.apply(pose));
          CatzRobotTracker.getInstance().setTrajectorySetpoint(AllianceFlipUtil.apply(pose));
        });
  }





    /** Getter for final autonomous Program */
    public Command getCommand() { 
        return lastRoutine.command();
    }


    //---------------------------------------------------------------------------------------------------------
    //
    //          Record Types
    //
    //---------------------------------------------------------------------------------------------------------
    /** A customizable auto routine associated with a single command. */
    private static final record AutoProgram(String name, List<AutoQuestion> questions, Command command) {}

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