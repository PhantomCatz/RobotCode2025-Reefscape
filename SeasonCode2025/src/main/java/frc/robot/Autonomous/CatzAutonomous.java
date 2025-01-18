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
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.Autonomous.CatzAutonomous.AutoQuestion;
import frc.robot.Autonomous.CatzAutonomous.AutoQuestionResponse;
import frc.robot.Autonomous.CatzAutonomous.AutoScoringOptions;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.CharacterizationCmds.FeedForwardCharacterization;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization.Direction;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.WaitUntilPassX;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.JSONUtil;
import frc.robot.Utilities.LocalADStarAK;
import frc.robot.Utilities.SwitchableChooser;
import frc.robot.Utilities.VirtualSubsystem;

public class CatzAutonomous extends VirtualSubsystem{
    private final int MAX_QUESTIONS = 5;
    private static final String AUTO_STRING = "Auto";
    private final LoggedDashboardChooser<PathPlannerAuto> autoProgramChooser = new LoggedDashboardChooser<>(AUTO_STRING + "/Program");

    private HashMap<String, DashboardCmd> dashboardCmds = new HashMap<>();
    private File autosDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    private File choreoPathsDirectory = new File(Filesystem.getDeployDirectory(), "choreo");
    private File pathplannerPathsDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

    private PathPlannerAuto lastProgram;
    private JSONParser parser = new JSONParser();

    //---------------------------------------------------------------------------------------------------
    //  Auto Questions
    //-----------------------------------------------------------------------------------------------------
    private RobotContainer m_container;

    public CatzAutonomous(RobotContainer container) {
        this.m_container = container;

        // Path follwing setup
        CatzRobotTracker tracker = CatzRobotTracker.getInstance();

        //------------------------------------------------------------------------------------------------------------
        // Autonmous questionaire gui configurations
        // ORDER MATTERS! Register named commands first, AutoBuilder second, Trajectories and add autos to dashboard last
        //------------------------------------------------------------------------------------------------------------

        class testPrintCmd extends Command{
            Timer time = new Timer();

            @Override
            public void initialize(){
                time.reset();
                time.start();
            }

            @Override
            public void execute(){
                System.out.println("hi");
            }

            @Override
            public void end(boolean interrupted){
                System.out.println("end");
            }

            @Override
            public boolean isFinished(){
                return time.get() > 2;
            }
        }

        HashMap<String, Command> testCommandChoices = new HashMap<>();
        testCommandChoices.put("TestLoopPrint", new testPrintCmd());
        dashboardCmds.put("print", new DashboardCmd("print", testCommandChoices));

        BooleanSupplier shouldFlip = ()-> AllianceFlipUtil.shouldFlipToRed();
        AutoBuilder.configure(
            tracker::getEstimatedPose,
            tracker::resetPose,
            tracker::getRobotChassisSpeeds,
            container.getCatzDrivetrain()::drive,
            DriveConstants.PATH_FOLLOWING_CONTROLLER,
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
            try {
                NamedCommands.registerCommand(pathName, new TrajectoryDriveCmd(PathPlannerPath.fromChoreoTrajectory(pathName), m_container.getCatzDrivetrain()));
            } catch (FileVersionException | IOException | ParseException e) {
                e.printStackTrace();
            }
        }
        
        for(File pathFile : pathplannerPathsDirectory.listFiles()){
            //to get rid of the extensions trailing the path names
            String pathName = pathFile.getName().replaceFirst("[.][^.]+$", ""); 
            try {
                NamedCommands.registerCommand(pathName, new TrajectoryDriveCmd(PathPlannerPath.fromPathFile(pathName), m_container.getCatzDrivetrain()));
            } catch (FileVersionException | IOException | ParseException e) {
                e.printStackTrace();
            }
        }

        for(String question: dashboardCmds.keySet()){
            NamedCommands.registerCommand(question, dashboardCmds.get(question));
        }

        for (File autoFile: autosDirectory.listFiles()){
            String autoName = autoFile.getName().replaceFirst("[.][^.]+$", "");
            autoProgramChooser.addDefaultOption(autoName, new PathPlannerAuto(autoName));
        }
    }

    @Override
    public void periodic() {
        // Update the list of questions
        PathPlannerAuto selectedProgram = autoProgramChooser.get();
        if (
            selectedProgram == null || 
            selectedProgram.equals(lastProgram) || 
            (DriverStation.isAutonomousEnabled() && lastProgram != null)
        ) {
            return;
        }

        try {
            for(int i=1; i<=MAX_QUESTIONS; i++){
                String questionName = "Question #" + String.valueOf(i);
                SmartDashboard.putString(questionName, "");
                SmartDashboard.putData(questionName + " Response", new SendableChooser<Command>());
            }

            JSONObject json = (JSONObject) parser.parse(new FileReader(autosDirectory + "/" + selectedProgram.getName() + ".auto"));
            ArrayList<Object> commands = JSONUtil.getCommandsFromPath(json);
            int questionCounter = 1;
           
            for(Object o : commands){
                String commandName = JSONUtil.getCommandName(o);
                DashboardCmd modifiableCommand = dashboardCmds.get(commandName);

                if(modifiableCommand != null){
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

    //---------------------------------------------------------------------------------------------------------
    //
    //          Pathfinding
    //
    //---------------------------------------------------------------------------------------------------------

    public Command getPathfindingCommand(Pose2d goal){
        Translation2d robotPos = CatzRobotTracker.getInstance().getEstimatedPose().getTranslation();
        if(AllianceFlipUtil.shouldFlipToRed()){
            robotPos = FlippingUtil.flipFieldPosition(robotPos);
        }
        Pathfinding.setStartPosition(robotPos);
        Pathfinding.setGoalPosition(goal.getTranslation());
        PathPlannerPath path = Pathfinding.getCurrentPath(DriveConstants.PATHFINDING_CONSTRAINTS, new GoalEndState(0, goal.getRotation()));

        if(path == null){
            return new InstantCommand();
        }else{
            return new TrajectoryDriveCmd(path, m_container.getCatzDrivetrain());
        }
    }

    public Command pathfindThenFollowPath(AutoScoringOptions option){ 
        Pose2d goal = new Pose2d(0, 0, new Rotation2d());
        return new SequentialCommandGroup(
            getPathfindingCommand(goal),
            new TrajectoryDriveCmd(Pathfinding.getCurrentPath(DriveConstants.PATHFINDING_CONSTRAINTS, new GoalEndState(0, goal.getRotation())), m_container.getCatzDrivetrain())
        );
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
        THINKING_ON_YOUR_FEET,
        IMMEDIATELY,
        SIX_SECONDS,
        LAST_SECOND,
        YES,
        NO
    }

    public static enum AutoScoringOptions {
        CORAL_ROD_1,
        CORAL_ROD_2,
        CORAL_ROD_4,
        CORAL_ROD_5,
        CORAL_ROD_7,
        CORAL_ROD_8,
        CORAL_ROD_10,
        CORAL_ROD_11,
    }


}