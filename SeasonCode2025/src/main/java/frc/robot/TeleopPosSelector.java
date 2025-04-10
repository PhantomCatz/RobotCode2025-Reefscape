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
package frc.robot;

import java.util.Arrays;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.Reef;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.CornerTrackingPathfinder;
import frc.robot.CatzSubsystems.CatzSuperstructure.CoralState;
import frc.robot.CatzSubsystems.CatzSuperstructure.Gamepiece;
import frc.robot.CatzSubsystems.CatzSuperstructure.LeftRight;
import frc.robot.CatzSubsystems.CatzStateCommands;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.QueueLEDState;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;

@SuppressWarnings({ "rawtypes", "unchecked" })
public class TeleopPosSelector { //TODO split up the file. it's too big and does too much stuff
  private final RobotContainer m_container;

  private final CommandXboxController xboxAux;

  private final String REEFSIDE = "Reefside ";
  private final String QUEUE = "PathQueue ";
  private final int NUM_QUEUE_DISPLAY = 4;
  private final double SELECTION_THRESHOLD = 0.7;

  private CatzSuperstructure superstructure;
  private CatzDrivetrain drivetrain;
  private CatzOuttake outtake;
  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();

  private Command currentDrivetrainCommand = new InstantCommand();
  private Command currentAutoplayCommand = new InstantCommand();
  private CornerTrackingPathfinder pathfinder = new CornerTrackingPathfinder();
  private Pair<Integer, LeftRight> currentPathfindingPair = new Pair<Integer, LeftRight>(0, LeftRight.LEFT);
  private Pair<Integer, LeftRight> currentlySelected = new Pair<Integer, LeftRight>(0, LeftRight.LEFT);
  private Deque<Pair<Pair<Integer, LeftRight>, Integer>> queuedPaths = new LinkedList<>();
  private HashMap<String, String> poseToLetter = new HashMap<>();

  private final boolean manualOverrideUseFakeCoral = false; //if the real robot doesn't have mechanisms to hold real coral, simulate one
  public final boolean useFakeCoral = manualOverrideUseFakeCoral || Robot.isSimulation();

  private boolean leftCoralStation = true;
  private boolean rightCoralStation = true;

  public boolean hasCoralSIM = true;

  private boolean isNetAiming = false;
  private boolean isNBALeftRight = false;

  public enum CancellationSource{
    NBA,
    NET,
    AQUA,
    NA
  }

  public TeleopPosSelector(CommandXboxController aux, RobotContainer container) {
    this.xboxAux = aux;
    this.m_container = container;
    this.outtake = container.getCatzOuttake();
    this.currentDrivetrainCommand.addRequirements(container.getCatzDrivetrain());
    this.currentDrivetrainCommand.addRequirements(container.getCatzElevator());
    this.currentDrivetrainCommand.addRequirements(container.getCatzOuttake());
    //this.currentAutoplayCommand.addRequirements(container.getCatzDrivetrain());

    superstructure = m_container.getSuperstructure();
    drivetrain = m_container.getCatzDrivetrain();

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

  public void toggleLeftStation() {
    leftCoralStation = !leftCoralStation;
    SmartDashboard.putBoolean("Left Coral Station", leftCoralStation);
  }

  public void toggleRightStation() {
    rightCoralStation = !rightCoralStation;
    SmartDashboard.putBoolean("Right Coral Station", rightCoralStation);
  }

  /**
   * When selecting the "best" coral station, the enabled coral station takes priority.
   * If the coral stations are both enabled or disabled, then it selects the closest one.
   * @return
   */
  public Pose2d getBestCoralStation() {
    Pose2d right = FieldConstants.CoralStation.getRightStation();
    Pose2d left = FieldConstants.CoralStation.getLeftStation();
    // TODO flip right and left when on red alliance

    if (!leftCoralStation && rightCoralStation) {
      return right;
    } else if (!rightCoralStation && leftCoralStation) {
      return left;
    } else {
      Pose2d nearestStation = right;
      Translation2d robotPos = tracker.getEstimatedPose().getTranslation();

      if (robotPos.getDistance(right.getTranslation()) > robotPos.getDistance(left.getTranslation())) {
        nearestStation = left;
      }
      return nearestStation;
    }
  }

  public void pathQueueAddFront(Pair<Integer, LeftRight> pose, int reefLevel) {
    // there was no joystick input
    if (pose == null || queuedPaths.size() >= NUM_QUEUE_DISPLAY)
      return;

    queuedPaths.addFirst(new Pair(pose, reefLevel));
  }

  public void pathQueueAddBack(Pair<Integer, LeftRight> pose, int reefLevel) {
    // there was no joystick input
    if (pose == null || queuedPaths.size() >= NUM_QUEUE_DISPLAY)
      return;

    queuedPaths.addLast(new Pair(pose, reefLevel));
  }

  public Pair<Pair<Integer, LeftRight>, Integer> pathQueuePeekFront() {
    if (queuedPaths.peekFirst() != null) {
      return queuedPaths.peekFirst();
    } else {
      return new Pair(null, 0);
    }
  }

  public Pair<Pair<Integer, LeftRight>, Integer> pathQueuePopFront() {
    if (queuedPaths.peekFirst() != null) {
      return queuedPaths.pollFirst();
    } else {
      return new Pair(null, 0);
    }
  }

  public void pathQueuePopBack() {
    queuedPaths.pollLast();
  }

  public void pathQueueClear() {
    queuedPaths.clear();
  }

  public void updateCurrentlySelected() {
    currentlySelected = getXBoxReefPos();

    //Update the display to see which branch is currently being selected by the aux driver.
    for (int side = 0; side < 6; side++) {
      SmartDashboard.putBoolean(REEFSIDE + side + " L",
          side == currentlySelected.getFirst() && currentlySelected.getSecond().equals(LeftRight.LEFT));
      SmartDashboard.putBoolean(REEFSIDE + side + " R",
          side == currentlySelected.getFirst() && currentlySelected.getSecond().equals(LeftRight.RIGHT));
    }

    //clear the display queue in case a queue was deleted
    for (int i = 0; i < NUM_QUEUE_DISPLAY; i++) {
      SmartDashboard.putString(QUEUE + i, "");
    }

    //Update the queue display
    int i = 0;
    for (Pair<Pair<Integer, LeftRight>, Integer> pair : queuedPaths) {
      Pair<Integer, LeftRight> pose = pair.getFirst();
      SmartDashboard.putString(QUEUE + i,
          poseToLetter.get(pose.getFirst() + " " + pose.getSecond()) + " " + pair.getSecond());
      i++;
    }

    int ledQueue = 0;
    for (Pair<Pair<Integer, LeftRight>, Integer> pair : queuedPaths) {
      ledQueue++;
    }

    if(ledQueue == 1) {
      CatzLED.getInstance().setQueueLEDState(QueueLEDState.ONE_CORAL);
    } else if(ledQueue == 2) {
      CatzLED.getInstance().setQueueLEDState(QueueLEDState.TWO_CORAL);
    } else if(ledQueue == 3) {
      CatzLED.getInstance().setQueueLEDState(QueueLEDState.THREE_CORAL);
    } else if(ledQueue == 4) {
      CatzLED.getInstance().setQueueLEDState(QueueLEDState.FOUR_CORAL);
    } else {
      CatzLED.getInstance().setQueueLEDState(QueueLEDState.EMPTY);
    }

  }

  public Pair<Integer, LeftRight> getXBoxReefPos() {
    int side = 0;
    LeftRight leftRight = LeftRight.LEFT;

    //The x and y are field relative with blue alliance origin.
    final double x = -xboxAux.getRightY();
    final double y = -xboxAux.getRightX();

    if (Math.hypot(x, y) > SELECTION_THRESHOLD) {
      // ensures angle is between 0-2pi (Dr. Eric Yuchen Lu (MD)'s idea)
      double angle = (Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI);
      side = (int) Math.round(angle * 3.0 / Math.PI) % 6; //mod 6 to keep the side between 0 and 5

      // Split the angle into twelveth and determine the left or right. Left and right changes when x is negative
      int leftOrRight = (int) Math.ceil(angle * 6.0 / Math.PI) % 12;
      if (x > 0) {
        leftRight = leftOrRight % 2 == 0 ? LeftRight.RIGHT : LeftRight.LEFT;
      } else {
        leftRight = leftOrRight % 2 == 0 ? LeftRight.LEFT : LeftRight.RIGHT;
      }
      return new Pair<Integer, LeftRight>(side, leftRight);
    } else {
      return currentlySelected;
    }
  }

  /**
   * Iterate through all reef positions and find the closest one.
   * @return
   */
  public Pair<Pair<Integer, LeftRight>, Integer> getClosestReefPos() {
    int closestSide = 0;
    LeftRight closestLeftRight = LeftRight.LEFT;
    Translation2d robotPos = tracker.getEstimatedPose().getTranslation();

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
    return new Pair(new Pair(closestSide, closestLeftRight), superstructure.getLevel());
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

    Translation2d scoringPos = radius.plus(Reef.center);

    if(superstructure.getChosenGamepiece() == Gamepiece.CORAL){
      Translation2d leftRight = unitLeftRight.times(leftRightPos.NUM * Reef.leftRightDistance);
      if (unitLeftRight.getY() < 0) {
        leftRight = leftRight.times(-1);
      }

      scoringPos.plus(leftRight);
    }

    return AllianceFlipUtil.apply(new Pose2d(scoringPos, selectedAngle.plus(Rotation2d.k180deg)));
  }

  public Pose2d calculateReefPose(Pair<Integer, LeftRight> pair, boolean isNBA, boolean isDistanced) {
    if (isNBA) {
      currentPathfindingPair = pair;
    }
    if (pair == null) {
      return null;
    } else {
      return calculateReefPose(pair.getFirst(), pair.getSecond(), isDistanced);
    }
  }



  public PathPlannerPath getClosestNetPath(){
    PathPlannerPath path = pathfinder.getPathToNet(CatzRobotTracker.getInstance().getEstimatedPose().getTranslation(), new GoalEndState(0.0, AllianceFlipUtil.apply(Rotation2d.kZero)));
    if(path == null){
      return null;
    }

    if (AllianceFlipUtil.shouldFlipToRed()) {
      path = path.flipPath();
    }

    return path;
  }

  public Command runToNetCommand(){
    return new InstantCommand(() -> {
      isNetAiming = true;
      currentDrivetrainCommand.cancel();
      PathPlannerPath path = getClosestNetPath();
      if(path != null){
        currentDrivetrainCommand = new TrajectoryDriveCmd(path, drivetrain, true, m_container, true);
      }else{
      currentDrivetrainCommand = new InstantCommand();
      }
      currentDrivetrainCommand.schedule();
    });
  }

  public PathPlannerPath getPathfindingPath(Supplier<Pose2d> goalSupplier) {
    Pose2d goal = goalSupplier.get();
    if (goal == null) {
      System.out.println("The goal is null");
      return null;
    }

    Translation2d robotPos = tracker.getEstimatedPose().getTranslation();
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

    Translation2d robotPos = tracker.getEstimatedPose().getTranslation();
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

  public void runLeftRight(LeftRight leftRight){
    if(isNetAiming) {
      runLeftRightNet(leftRight);
    }else{
      runLeftRightNBA(leftRight);
    }
  }

  private void runLeftRightNBA(LeftRight leftRight) {
    if (currentPathfindingPair == null)
      return; // means it is in AQUA

    currentDrivetrainCommand.cancel();

    Pose2d currentPose = tracker.getEstimatedPose();
    Pose2d goal = calculateReefPose(new Pair<Integer, LeftRight>(currentPathfindingPair.getFirst(), leftRight), true, false); //TODO decide whether or not to have distanced

    Translation2d goalPos = goal.getTranslation();
    Translation2d currentPos = currentPose.getTranslation();
    Translation2d direction = goalPos.minus(currentPos).div(2.0);

    Logger.recordOutput("LeftRightGoal", goal);

    // if (direction.getNorm() <= 1e-3) {
    //   return;
    // }
    isNBALeftRight = true;

    PathPlannerPath path = getStraightLinePath(currentPose, goal, DriveConstants.PATHFINDING_CONSTRAINTS); //TODO might need to scale constraints based off of distance from reef?


    currentDrivetrainCommand = new TrajectoryDriveCmd(path, drivetrain, true, m_container, true).andThen(m_container.rumbleDrvAuxController(1.0, 0.2)).andThen(new InstantCommand(() -> isNBALeftRight = false));

    try{
      currentDrivetrainCommand.schedule();
    }catch(Error e){
      e.printStackTrace();
      //i dunno man spamming nba causes so much problems
    }
  }

  public PathPlannerPath getMoveScorePath(){
    Pose2d goalPose = calculateReefPose(getClosestReefPos().getFirst(), true, false);

    return getStraightLinePath(CatzRobotTracker.getInstance().getEstimatedPose(), goalPose, DriveConstants.PATHFINDING_CONSTRAINTS);
  }

  private void runLeftRightNet(LeftRight leftRight){
    currentDrivetrainCommand.cancel();

    Pose2d currentPose = tracker.getEstimatedPose();
    Pose2d goal;

    if(leftRight == LeftRight.LEFT){
      goal = new Pose2d(FieldConstants.Net.getX(), FieldConstants.Net.getYLeft(), AllianceFlipUtil.apply(Rotation2d.kZero));
    }else{
      goal = new Pose2d(FieldConstants.Net.getX(), FieldConstants.Net.getYRight(), AllianceFlipUtil.apply(Rotation2d.kZero));
    }

    PathPlannerPath path = getStraightLinePath(currentPose, goal, DriveConstants.LEFT_RIGHT_NET_CONSTRAINTS);
    currentDrivetrainCommand = new TrajectoryDriveCmd(path, drivetrain, false, m_container, true);
    currentDrivetrainCommand.schedule();
  }

  //------------------------------------------------------------------------------------
  //
  //  Xbox Commands
  //
  //-----------------------------------------------------------------------------------

  public Command runAutoCommand() {
    return new InstantCommand(() -> {
      currentAutoplayCommand.cancel();
      currentAutoplayCommand = new Command() {

        @Override
        public void initialize() {
          currentDrivetrainCommand.cancel();
          currentDrivetrainCommand = getNextCommand();
          currentDrivetrainCommand.schedule();
        }

        @Override
        public void execute() {
          if (!currentDrivetrainCommand.isScheduled() == true) { //haha L
            currentDrivetrainCommand = getNextCommand();
            currentDrivetrainCommand.schedule();
          }
        }

        @Override
        public boolean isFinished() {
          return false;
        }

        @Override
        public void end(boolean interrupted) {
          currentDrivetrainCommand.cancel();
        }

      };
      currentAutoplayCommand.schedule();
    });
  }

  private Command getNextCommand() {
    Pair<Pair<Integer, LeftRight>, Integer> pair = pathQueuePeekFront();

    if (useFakeCoral) {
      if (hasCoralSIM) {
        if (queuedPaths.isEmpty()){
          return new InstantCommand();
        }
        return getReefScoreCommand(pair).andThen(new InstantCommand(() -> pathQueuePopFront()));
      } else {
        return getCoralStationCommand();
      }
    } else {
      if (outtake.isDesiredCoralState(true)) {
        return getCoralStationCommand();
      } else {
        System.out.println("i have coral/?");
          if (queuedPaths.isEmpty()) {System.out.println("no ");return new InstantCommand();}
        return getReefScoreCommand(pair).andThen(new InstantCommand(() -> pathQueuePopFront()));
      }
    }
  }

  @SuppressWarnings("static-access")
  public Command runToNearestBranch() {

    return new InstantCommand(() -> {
      isNetAiming = false;
      currentPathfindingPair = getClosestReefPos().getFirst();
      currentDrivetrainCommand.cancel();
      try{
        //TODO add a check to see if the robot is against the wall but angled so that it runs distanced scoring
        Command prematureCommand;//superstructure.getChosenGamepiece() == Gamepiece.CORAL ? CatzStateCommands.LXElevator(m_container, superstructure.getLevel()) : CatzStateCommands.XAlgae(m_container, superstructure.getLevel());
        Pair<Integer, LeftRight> closestPos = getClosestReefPos().getFirst();

        if(superstructure.getChosenGamepiece() ==  Gamepiece.CORAL){
          prematureCommand = CatzStateCommands.LXElevator(m_container, superstructure.getLevel());
        }else{ //selected game piece is algae
          if(closestPos.getFirst() % 2 == 0){
            prematureCommand = CatzStateCommands.botAlgae(m_container);
          }else{
            prematureCommand = CatzStateCommands.topAlgae(m_container);
          }
        }
        // PathPlannerPath path = getPathfindingPath(calculateReefPose(getClosestReefPos().getFirst(), true, false));
        PathPlannerPath path = getStraightLinePath(tracker.getEstimatedPose(), calculateReefPose(closestPos, true, false), DriveConstants.PATHFINDING_CONSTRAINTS);

        currentDrivetrainCommand = new TrajectoryDriveCmd(path, m_container.getCatzDrivetrain(), true, m_container, true)
                                        .deadlineFor(new RepeatCommand(prematureCommand.onlyIf(() -> drivetrain.getDistanceError() < DriveConstants.PREDICT_DISTANCE_SCORE)))
                                        .andThen(m_container.controllerRumbleCommand());
        currentDrivetrainCommand.schedule();
      }catch(Exception e){
        e.printStackTrace();
      }
    });
  }

  public Command runCoralStationCommand() {
    return new InstantCommand(() -> {
      currentPathfindingPair = null;
      currentDrivetrainCommand.cancel();
      currentDrivetrainCommand = getCoralStationCommand();
      currentDrivetrainCommand.schedule();
    });
  }

  public Command getCoralStationCommand() {
    return CatzStateCommands.driveToCoralStation(m_container, getPathfindingPath(getBestCoralStation()), ()->true);
  }

  public Command getReefScoreCommand(Pair<Pair<Integer, LeftRight>, Integer> pair) {
    currentPathfindingPair = null;
    isNetAiming = false;

    return CatzStateCommands.driveToScore(
      m_container,
      getPathfindingPath(calculateReefPose(pair.getFirst(), false, false)),
      pair.getSecond()
    );
  }

  public Command cancelCurrentDrivetrainCommand(CancellationSource source) {
    return new InstantCommand(() -> {
      if(source == CancellationSource.NBA){
        isNBALeftRight = false;
      }
      if(isNBALeftRight) {
        return;
      } //you want to be able to cancel leftright with NBA

      currentDrivetrainCommand.cancel();
    });
  }

  public Command cancelAutoCommand() {
    return new InstantCommand(() -> {
      currentAutoplayCommand.cancel();
    });
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
