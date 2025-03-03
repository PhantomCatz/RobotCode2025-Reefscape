// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.Reef;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.CornerTrackingPathfinder;
import frc.robot.CatzSubsystems.CatzSuperstructure.LeftRight;
import frc.robot.CatzSubsystems.CatzSuperstructure.RobotAction;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED.QueueLEDState;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.Commands.DriveAndRobotOrientationCmds.DriveAndCycle;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;

@SuppressWarnings({ "rawtypes", "unchecked" })
public class TeleopPosSelector {
  private final RobotContainer m_container;

  private final CommandXboxController xboxAux;

  private final String REEFSIDE = "Reefside ";
  private final String QUEUE = "PathQueue ";
  private final int NUM_QUEUE_DISPLAY = 4;
  private final double SELECTION_THRESHOLD = 0.3;
  private final double ELEVATOR_RAISE_DIST = 1.0; // meters

  private CatzSuperstructure superstructure;
  private CatzDrivetrain drivetrain;
  private CatzOuttake outtake;
  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();

  private Command currentDrivetrainCommand = new InstantCommand();
  private Command currentAutoplayCommand = new InstantCommand();
  private CornerTrackingPathfinder pathfinder = new CornerTrackingPathfinder();
  private Pair<Integer, LeftRight> currentPathfindingPair = new Pair<Integer, LeftRight>(0, LeftRight.LEFT);
  private Deque<Pair<Pair<Integer, LeftRight>, Integer>> queuedPaths = new LinkedList<>();
  private HashMap<String, String> poseToLetter = new HashMap<>();

  private final boolean manualOverrideUseFakeCoral = false; //if the real robot doesn't have mechanisms to hold real coral, simulate one
  public final boolean useFakeCoral = manualOverrideUseFakeCoral || Robot.isSimulation();

  private boolean leftCoralStation = true;
  private boolean rightCoralStation = true;

  public boolean hasCoralSIM = true;

  public TeleopPosSelector(CommandXboxController aux, RobotContainer container) {
    this.xboxAux = aux;
    this.m_container = container;
    this.outtake = container.getCatzOuttake();
    this.currentDrivetrainCommand.addRequirements(container.getCatzDrivetrain());
    //this.currentAutoplayCommand.addRequirements(container.getCatzDrivetrain());

    superstructure = m_container.getSuperstructure();
    drivetrain = m_container.getCatzDrivetrain();

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

  public void toggleLeftStation() {
    leftCoralStation = !leftCoralStation;
    SmartDashboard.putBoolean("Left Coral Station", leftCoralStation);
  }

  public void toggleRightStation() {
    rightCoralStation = !rightCoralStation;
    SmartDashboard.putBoolean("Right Coral Station", rightCoralStation);
  }

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
    Pair<Integer, LeftRight> currentlySelected = getXBoxReefPos();

    // there was no joystick selection, so display NBA
    if (currentlySelected == null) {
      currentlySelected = getClosestReefPos().getFirst();
    }

    for (int side = 0; side < 6; side++) {
      SmartDashboard.putBoolean(REEFSIDE + side + " L",
          side == currentlySelected.getFirst() && currentlySelected.getSecond().equals(LeftRight.LEFT));
      SmartDashboard.putBoolean(REEFSIDE + side + " R",
          side == currentlySelected.getFirst() && currentlySelected.getSecond().equals(LeftRight.RIGHT));
    }

    for (int i = 0; i < NUM_QUEUE_DISPLAY; i++) {
      SmartDashboard.putString(QUEUE + i, "");
    }

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

    final double x = -xboxAux.getRightY();
    final double y = -xboxAux.getRightX();

    if (Math.hypot(x, y) > SELECTION_THRESHOLD) {
      // ensures angle is between 0-2pi (Dr. Eric Yuchen Lu (MD)'s idea)
      double angle = (Math.atan2(y, x) + 2 * Math.PI) % (2 * Math.PI);
      side = (int) Math.round(angle * 3.0 / Math.PI) % 6;

      // if angle is too close to 2pi, then it will return 12, but we want selected to
      // be between 0-11 (Dr. Eric Yuchen Lu (MD)'s idea)
      int leftOrRight = (int) Math.ceil(angle * 6.0 / Math.PI) % 12;
      if (x > 0) {
        leftRight = leftOrRight % 2 == 0 ? LeftRight.RIGHT : LeftRight.LEFT;
      } else {
        leftRight = leftOrRight % 2 == 0 ? LeftRight.LEFT : LeftRight.RIGHT;
      }
    } else {
      return null;
    }
    return new Pair<Integer, LeftRight>(side, leftRight);
  }

  public Pair<Pair<Integer, LeftRight>, Integer> getClosestReefPos() {
    int closestSide = 0;
    LeftRight closestLeftRight = LeftRight.LEFT;
    Translation2d robotPos = tracker.getEstimatedPose().getTranslation();

    for (int side = 0; side < 6; side++) {
      for (LeftRight leftRight : LeftRight.values()) {
        Pose2d reefPos = calculateReefPose(side, leftRight);
        if (reefPos.getTranslation().getDistance(robotPos) < calculateReefPose(closestSide, closestLeftRight)
            .getTranslation().getDistance(robotPos)) {
          closestSide = side;
          closestLeftRight = leftRight;
        }
      }
    }

    return new Pair(new Pair(closestSide, closestLeftRight), superstructure.getLevel());
  }

  // TODO calculate this on comptime and store it in an array
  public Pose2d calculateReefPose(int reefAngle, LeftRight leftRightPos) {
    Rotation2d selectedAngle = Rotation2d.fromRotations(reefAngle / 6.0);

    Translation2d unitRadius = new Translation2d(selectedAngle.getCos(), selectedAngle.getSin());
    Translation2d unitLeftRight = unitRadius.rotateBy(Rotation2d.fromDegrees(90));

    Translation2d radius = unitRadius.times(Reef.reefOrthogonalRadius + Reef.scoringDistance);
    // radius = radius.plus(unitLeftRight.times(-Units.inchesToMeters(0)));

    Translation2d leftRight = unitLeftRight.times(leftRightPos.NUM * Reef.leftRightDistance);
    if (unitLeftRight.getY() < 0) {
      leftRight = leftRight.times(-1);
    }

    Translation2d scoringPos = radius.plus(leftRight).plus(Reef.center);
    return AllianceFlipUtil.apply(new Pose2d(scoringPos, selectedAngle.plus(Rotation2d.k180deg)));
  }

  public Pose2d calculateReefPose(Pair<Integer, LeftRight> pair, boolean isNBA) {
    if (isNBA) {
      currentPathfindingPair = pair;
    }
    if (pair == null) {
      return null;
    } else {
      return calculateReefPose(pair.getFirst(), pair.getSecond());
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

  public void runLeftRight(LeftRight leftRight) {
    if (currentPathfindingPair == null)
      return; // means it is in AQUA

    currentDrivetrainCommand.cancel();

    Pose2d goal = calculateReefPose(new Pair<Integer, LeftRight>(currentPathfindingPair.getFirst(), leftRight), true);
    Pose2d currentPose = tracker.getEstimatedPose();

    Translation2d goalPos = goal.getTranslation();
    Translation2d currentPos = currentPose.getTranslation();
    Translation2d direction = goalPos.minus(currentPos).div(2.0);

    //if too far from reef side, then don't NBA
    if (currentPose.getTranslation().getDistance(goal.getTranslation()) > Reef.leftRightDistance * 3
        || direction.getNorm() <= 1e-3) {
      return;
    }

    PathPlannerPath path = new PathPlannerPath(
        Arrays.asList(new Waypoint[] {
            new Waypoint(null, currentPos, currentPos.plus(direction)),
            new Waypoint(goalPos.minus(direction), goalPos, null)
        }),
        DriveConstants.PATHFINDING_CONSTRAINTS,
        null,
        new GoalEndState(0, goal.getRotation()));

    if (AllianceFlipUtil.shouldFlipToRed()) {
      path = path.flipPath();
    }

    currentDrivetrainCommand = new TrajectoryDriveCmd(path, drivetrain, true, m_container);
    currentDrivetrainCommand.schedule();
  }

  public void runLeftRightShift(LeftRight leftRight) {
      if (currentPathfindingPair == null)
        return;

      Pose2d currentPose = tracker.getEstimatedPose();
      Rotation2d angle = Rotation2d.fromRotations(currentPathfindingPair.getFirst()/6.0);
      Translation2d direction = new Translation2d(angle.getCos(),angle.getSin());
      direction = direction.rotateBy(Rotation2d.fromDegrees(90));

      Translation2d currentPos = currentPose.getTranslation();

      if(leftRight == LeftRight.LEFT){
        direction = direction.times(-1);
      }

      if(currentPathfindingPair.getFirst() == 5 ||currentPathfindingPair.getFirst() == 0 ||currentPathfindingPair.getFirst() == 0){
        direction = direction.times(-1);
      }

      Translation2d goalPos = currentPos.plus(direction.times(10));

      PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints( // 540 // 720
        1.0,
        DriveConstants.DRIVE_CONFIG.maxLinearAcceleration(), // max vel causing messup
        DriveConstants.DRIVE_CONFIG.maxAngularVelocity(),
        DriveConstants.DRIVE_CONFIG.maxAngularAcceleration()
      );

      PathPlannerPath path = new PathPlannerPath(
        Arrays.asList(new Waypoint[] {
            new Waypoint(null, currentPos, currentPos.plus(direction)),
            new Waypoint(goalPos.minus(direction), goalPos, null)
        }),
        PATHFINDING_CONSTRAINTS,
        null,
        new GoalEndState(0, currentPos.getAngle()));

      if (AllianceFlipUtil.shouldFlipToRed()) {
        path = path.flipPath();
      }

      currentDrivetrainCommand = new TrajectoryDriveCmd(path, drivetrain, true, m_container);
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
      if (outtake.hasCoral()) {
        if (queuedPaths.isEmpty())
          return new InstantCommand();
        return getReefScoreCommand(pair).andThen(new InstantCommand(() -> pathQueuePopFront()));
      } else {
        return getCoralStationCommand();
      }
    }

  }

  public Command runToNearestBranch() {
    return new InstantCommand(() -> {
      currentPathfindingPair = getClosestReefPos().getFirst();
      currentDrivetrainCommand.cancel();
      currentDrivetrainCommand = new TrajectoryDriveCmd(getPathfindingPath(calculateReefPose(currentPathfindingPair, true)), drivetrain, true, m_container);
      currentDrivetrainCommand.schedule();
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
    return new DriveAndCycle(getPathfindingPath(getBestCoralStation()), m_container, RobotAction.INTAKE);
  }

  public Command getReefScoreCommand(Pair<Pair<Integer, LeftRight>, Integer> pair) {
    currentPathfindingPair = null;
    return new DriveAndCycle(getPathfindingPath(calculateReefPose(pair.getFirst(), false)), m_container, RobotAction.OUTTAKE, pair.getSecond());
  }

  public Command cancelCurrentDrivetrainCommand() {
    return new InstantCommand(() -> {
      currentDrivetrainCommand.cancel();
    });
  }

  public Command cancelAutoCommand() {
    return new InstantCommand(() -> {
      currentAutoplayCommand.cancel();
    });
  }
}
