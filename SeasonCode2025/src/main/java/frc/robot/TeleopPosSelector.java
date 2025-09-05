package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.FieldConstants.Reef;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.CornerTrackingPathfinder;
import frc.robot.CatzSubsystems.CatzSuperstructure.Gamepiece;
import frc.robot.CatzSubsystems.CatzSuperstructure.LeftRight;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;

@SuppressWarnings({ "rawtypes", "unchecked" })
public class TeleopPosSelector { //TODO split up the file. it's too big and does too much stuff
  public static final TeleopPosSelector Instance = new TeleopPosSelector();

  private CornerTrackingPathfinder pathfinder = new CornerTrackingPathfinder();
  public Pair<Integer, LeftRight> recentNBAPos = new Pair<Integer, LeftRight>(0, LeftRight.LEFT);
  private HashMap<String, String> poseToLetter = new HashMap<>();

  private final boolean manualOverrideUseFakeCoral = false; //if the real robot doesn't have mechanisms to hold real coral, simulate one
  public final boolean useFakeCoral = manualOverrideUseFakeCoral || Robot.isSimulation();

  private boolean leftCoralStation = true;
  private boolean rightCoralStation = true;

  public boolean hasCoralSIM = true;

  public enum CancellationSource{
    NBA,
    NET,
    AQUA,
    NA
  }

  private TeleopPosSelector() {

    //Internally, we define a "branch" as one of the 6 sides of the reef. The more specific side of the branch is selected with the LEFT and RIGHT
    //The LEFT and RIGHT is always from the driverstation's POV

    //The side is defined with numbers starting at 0 to 5. 0 is the farthest side from the driverstation and the number increases counter clockwise (makes calculation easier)
    poseToLetter.put("0 RIGHT", "G");
    poseToLetter.put("0 LEFT", "H");
    poseToLetter.put("1 RIGHT", "I");
    poseToLetter.put("1 LEFT", "J");
    poseToLetter.put("2 RIGHT", "L");
    poseToLetter.put("2 LEFT", "K");
    poseToLetter.put("3 RIGHT", "B");
    poseToLetter.put("3 LEFT", "A");
    poseToLetter.put("4 RIGHT", "D");
    poseToLetter.put("4 LEFT", "C");
    poseToLetter.put("5 RIGHT", "E");
    poseToLetter.put("5 LEFT", "F");

    SmartDashboard.putBoolean("Left Coral Station", leftCoralStation);
    SmartDashboard.putBoolean("Right Coral Station", rightCoralStation);
  }

  public String getPoseToLetter(String pose){
    return poseToLetter.get(pose);
  }

  /**
   * Iterate through all reef positions and find the closest one.
   * @return
   */
  public Pair<Pair<Integer, LeftRight>, Integer> getClosestReefPos() {
    int closestSide = 0;
    LeftRight closestLeftRight = LeftRight.LEFT;
    Translation2d robotPos = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();

    for (int side = 0; side < 6; side++) {
      for (LeftRight leftRight : LeftRight.values()) {
        Pose2d reefPos = calculateReefPose(side, leftRight, false);
        if (reefPos.getTranslation().getDistance(robotPos) < calculateReefPose(closestSide, closestLeftRight, false)
            .getTranslation().getDistance(robotPos)) {
          closestSide = side;
          closestLeftRight = leftRight;
        }
      }
    }
    return new Pair(new Pair(closestSide, closestLeftRight), CatzSuperstructure.Instance.getLevel());
  }

  public Pair<Integer, LeftRight> getClosestReefPos(Translation2d pose) {
    int closestSide = 0;
    LeftRight closestLeftRight = LeftRight.LEFT;
    Translation2d robotPos = pose;

    for (int side = 0; side < 6; side++) {
      for (LeftRight leftRight : LeftRight.values()) {
        Pose2d reefPos = calculateReefPose(side, leftRight, false);
        if (reefPos.getTranslation().getDistance(robotPos) < calculateReefPose(closestSide, closestLeftRight, false)
            .getTranslation().getDistance(robotPos)) {
          closestSide = side;
          closestLeftRight = leftRight;
        }
      }
    }
    return new Pair(closestSide, closestLeftRight);
  }

  // TODO calculate this on comptime and store it in an array
  @SuppressWarnings("static-access")
  public Pose2d calculateReefPose(int reefAngle, LeftRight leftRightPos, boolean isDistanced) {
    Rotation2d selectedAngle = Rotation2d.fromRotations(reefAngle / 6.0);

    //A unit vector with its origin placed at the center of the reef pointing orthogonally to the selected side.
    Translation2d unitRadius = new Translation2d(selectedAngle.getCos(), selectedAngle.getSin());

    //Rotate the unit vector 90 degrees to the left to account for branch distances
    Translation2d unitLeftRight = unitRadius.rotateBy(Rotation2d.fromDegrees(90));

    //This vector is now pointing at the scoring position of the robot, but not accounting for the left right distances
    Translation2d radius = unitRadius.times(Reef.reefOrthogonalRadius + Reef.scoringDistance);

    if(isDistanced){
      radius = radius.plus(unitRadius.times(Reef.backDistance));
    }

    Translation2d leftRight = unitLeftRight.times(leftRightPos.NUM * Reef.leftRightDistance);

    //Left right changes depending on whether the selected side is the upper or lower half of the reef.
    if (unitLeftRight.getY() < 0) {
      leftRight = leftRight.times(-1);
    }

    if(CatzSuperstructure.Instance.getChosenGamepiece() == Gamepiece.ALGAE){
      leftRight = new Translation2d();
    }

    Translation2d scoringPos = radius.plus(leftRight).plus(Reef.center);
    return AllianceFlipUtil.apply(new Pose2d(scoringPos, selectedAngle.plus(Rotation2d.k180deg)));
  }

  public Pose2d calculateReefPose(Pair<Integer, LeftRight> pair, boolean isNBA, boolean isDistanced) {
    if (isNBA) {
      recentNBAPos = pair;
    }
    if (pair == null) {
      return null;
    } else {
      return calculateReefPose(pair.getFirst(), pair.getSecond(), isDistanced);
    }
  }



  public PathPlannerPath getClosestNetPath(){
    PathPlannerPath path = pathfinder.getPathToNet(CatzRobotTracker.Instance.getEstimatedPose().getTranslation(), new GoalEndState(0.0, AllianceFlipUtil.apply(Rotation2d.kZero)));
    if(path == null){
      return null;
    }

    if (AllianceFlipUtil.shouldFlipToRed()) {
      path = path.flipPath();
    }

    return path;
  }

  public PathPlannerPath getPathfindingPath(Supplier<Pose2d> goalSupplier) {
    Pose2d goal = goalSupplier.get();
    if (goal == null) {
      System.out.println("The goal is null");
      return null;
    }

    Translation2d robotPos = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
    PathPlannerPath path   = pathfinder.getPath(robotPos, goal.getTranslation(),
        new GoalEndState(0.0, goal.getRotation()));

    if (path == null) {
      System.out.println("The path is null: " + robotPos + goal.getTranslation());
      return null;
    } else {
      if (AllianceFlipUtil.shouldFlipToRed()) {
        path = path.flipPath();
      }
      return path;
    }
  }

  public PathPlannerPath getPathfindingPath(Pose2d goal) {
    if (goal == null) {
      System.out.println("The goal is null");
      return null;
    }

    Translation2d robotPos = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
    PathPlannerPath path   = pathfinder.getPath(robotPos, goal.getTranslation(),
        new GoalEndState(0.0, goal.getRotation()));

    if (path == null) {
      System.out.println("The path is null: " + robotPos + goal.getTranslation());
      return null;
    } else {
      if (AllianceFlipUtil.shouldFlipToRed()) {
        path = path.flipPath();
      }
      return path;
    }
  }

  public PathPlannerPath getPathfindingPath(Translation2d start, Pose2d goal) {
    if (goal == null) {
      System.out.println("The goal is null");
      return null;
    }

    Translation2d robotPos = start;
    PathPlannerPath path   = pathfinder.getPath(robotPos, goal.getTranslation(),
        new GoalEndState(0.0, goal.getRotation()));

    if (path == null) {
      System.out.println("The path is null: " + robotPos + goal.getTranslation());
      return null;
    } else {
      if (AllianceFlipUtil.shouldFlipToRed()) {
        path = path.flipPath();
      }
      return path;
    }
  }

  public Command runLeftRight(LeftRight leftRight){
    return new DeferredCommand(() -> {
      if(recentNBAPos == null) return new InstantCommand(); //the driver did not NBA yet, so there is no left right to go to

      Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
      Pose2d goal = calculateReefPose(new Pair<Integer, LeftRight>(recentNBAPos.getFirst(), leftRight), true, false); //TODO decide whether or not to have distanced

      Logger.recordOutput("LeftRightGoal", goal);

      PathPlannerPath path = getStraightLinePath(currentPose, goal, DriveConstants.PATHFINDING_CONSTRAINTS); //TODO might need to scale constraints based off of distance from reef?

      return new TrajectoryDriveCmd(path, true, true).andThen(RobotContainer.Instance.rumbleDrvAuxController(1.0, 0.2));
    }, Set.of(CatzDrivetrain.Instance));

  }

  public PathPlannerPath getMoveScorePath(){
    Pose2d goalPose = calculateReefPose(getClosestReefPos().getFirst(), true, false);

    return getStraightLinePath(CatzRobotTracker.Instance.getEstimatedPose(), goalPose, DriveConstants.PATHFINDING_CONSTRAINTS);
  }

  //------------------------------------------------------------------------------------
  //
  //  Xbox Commands
  //
  //-----------------------------------------------------------------------------------

  @SuppressWarnings("static-access")
  public Command runToNearestBranch() {

    return new DeferredCommand(() -> {
      recentNBAPos = getClosestReefPos().getFirst();

      // run elevator early
      Command prematureCommand;
      if(CatzSuperstructure.Instance.getChosenGamepiece() == Gamepiece.CORAL){
        prematureCommand = Commands.sequence(
            CatzElevator.Instance.setCanMoveElevator(true), 
            CatzSuperstructure.Instance.LXElevator(CatzSuperstructure.Instance.getLevel())
        );
      }else{ //algae
        prematureCommand = new InstantCommand();
      }

      PathPlannerPath path = getStraightLinePath(CatzRobotTracker.Instance.getEstimatedPose(), calculateReefPose(getClosestReefPos().getFirst(), true, false), DriveConstants.PATHFINDING_CONSTRAINTS);

      Command prepareScorePos = Commands.sequence(
                                    Commands.parallel(
                                      new TrajectoryDriveCmd(path, true, true),
                                      Commands.deadline(
                                        new RepeatCommand(prematureCommand.onlyIf(() -> CatzDrivetrain.Instance.getDistanceError() < DriveConstants.PREDICT_DISTANCE_SCORE))
                                      )
                                    ),
                                    RobotContainer.Instance.controllerRumbleCommand()
                                );

      return prepareScorePos;
    }, Set.of(CatzDrivetrain.Instance));
  }


  public PathPlannerPath getStraightLinePath(Pose2d start, Pose2d goal, PathConstraints constraints){
    Translation2d currentPose = start.getTranslation();
    Translation2d goalPos = goal.getTranslation();
    Translation2d direction = goalPos.minus(currentPose).div(2.0);

    PathPlannerPath path = new PathPlannerPath(
        Arrays.asList(new Waypoint[] {
            new Waypoint(null, currentPose, currentPose.plus(direction)),
            new Waypoint(goalPos.minus(direction), goalPos, null)
        }),
        constraints,
        null,
        new GoalEndState(0, goal.getRotation()));

    if (AllianceFlipUtil.shouldFlipToRed()) {
      path = path.flipPath();
    }

    return path;
  }
}
