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
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.Reef;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.CornerTrackingPathfinder;
import frc.robot.CatzSubsystems.CatzSuperstructure.LeftRight;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;

public class TeleopPosSelector extends SubsystemBase {
  private final RobotContainer m_container;

  private final CommandXboxController xboxAux;

  private final String REEFSIDE = "Reefside ";
  private final String QUEUE = "PathQueue ";
  private final int NUM_QUEUE_DISPLAY = 4;
  private final double SELECTION_THRESHOLD = 0.3;

  private CornerTrackingPathfinder pathfinder = new CornerTrackingPathfinder();
  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
  private Command currentPathfindingCommand = new InstantCommand();
  private Pair<Integer, LeftRight> currentPathfindingPair = new Pair<Integer, LeftRight>(0, LeftRight.LEFT);
  private Deque<Pair<Integer, LeftRight>> queuedPaths = new LinkedList<>();

  private HashMap<String, String> poseToLetter = new HashMap<>();

  public TeleopPosSelector(CommandXboxController aux, RobotContainer container) {
    this.xboxAux = aux;
    this.m_container = container;
    this.currentPathfindingCommand.addRequirements(container.getCatzDrivetrain());

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
  }

  public void pathQueueAddFront(Pair<Integer, LeftRight> pose) {
    // there was no joystick input
    if (pose == null && queuedPaths.size() <= NUM_QUEUE_DISPLAY)
      return;

    queuedPaths.addFirst(pose);
  }

  public void pathQueueAddBack(Pair<Integer, LeftRight> pose) {
    // there was no joystick input
    if (pose == null && queuedPaths.size() <= NUM_QUEUE_DISPLAY)
      return;

    queuedPaths.addLast(pose);
  }

  public Pair<Integer, LeftRight> pathQueuePeekFront() {
    return queuedPaths.peekFirst();
  }

  public void pathQueuePopFront() {
    queuedPaths.pollFirst();
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
      currentlySelected = getClosestReefPos();
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
    for (Pair<Integer, LeftRight> pair : queuedPaths) {
      SmartDashboard.putString(QUEUE + i, poseToLetter.get(pair.getFirst() + " " + pair.getSecond()));
      i++;
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

  public Pair<Integer, LeftRight> getClosestReefPos() {
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

    return new Pair<Integer, LeftRight>(closestSide, closestLeftRight);
  }

  public Pose2d calculateReefPose(int reefAngle, LeftRight leftRightPos) {
    Rotation2d selectedAngle = Rotation2d.fromRotations(reefAngle / 6.0);

    Translation2d unitRadius = new Translation2d(selectedAngle.getCos(), selectedAngle.getSin());
    Translation2d unitLeftRight = unitRadius.rotateBy(Rotation2d.fromDegrees(90));

    Translation2d radius = unitRadius.times(Reef.reefOrthogonalRadius + Reef.scoringDistance);
    Translation2d leftRight = unitLeftRight.times(leftRightPos.NUM * Reef.leftRightDistance);
    if (unitLeftRight.getY() < 0) {
      leftRight = leftRight.times(-1);
    }

    Translation2d scoringPos = radius.plus(leftRight).plus(Reef.center);
    return AllianceFlipUtil.apply(new Pose2d(scoringPos, selectedAngle));
  }

  public Pose2d calculateReefPose(Pair<Integer, LeftRight> pair) {
    if (pair == null) {
      return null;
    } else {
      return calculateReefPose(pair.getFirst(), pair.getSecond());
    }
  }

  public Command getPathfindingCommand(Pose2d goal) {
    if (goal == null) {
      return new InstantCommand();
    }

    Translation2d robotPos = tracker.getEstimatedPose().getTranslation();
    PathPlannerPath path = pathfinder.getPath(robotPos, goal.getTranslation(),
        new GoalEndState(0.0, goal.getRotation()));

    if (path == null) {
      return new InstantCommand();
    } else {
      if (AllianceFlipUtil.shouldFlipToRed()) {
        path = path.flipPath();
      }
      return new TrajectoryDriveCmd(path, m_container.getCatzDrivetrain(), true);
    }
  }

  public Command runLeftRightCommand(LeftRight leftRight) {
    return runOnce(() -> {
      Pose2d goal = calculateReefPose(new Pair<Integer, LeftRight>(currentPathfindingPair.getFirst(), leftRight));
      Pose2d currentPose = tracker.getEstimatedPose();

      Translation2d goalPos = goal.getTranslation();
      Translation2d currentPos = currentPose.getTranslation();
      Translation2d direction = goalPos.minus(currentPos).div(2.0);

      if (currentPose.getTranslation().getDistance(goal.getTranslation()) > Reef.leftRightDistance * 3
          || direction.getNorm() <= 1e-3) {
        return;
      }

      currentPathfindingCommand.cancel();
      currentPathfindingCommand = new TrajectoryDriveCmd(new PathPlannerPath(
        Arrays.asList(new Waypoint[] {
          new Waypoint(null, currentPos, currentPos.plus(direction)),
          new Waypoint(goalPos.minus(direction), goalPos, null)
        }),
        DriveConstants.PATHFINDING_CONSTRAINTS,
        null,
        new GoalEndState(0, goal.getRotation())
      ), m_container.getCatzDrivetrain(), true);
      currentPathfindingCommand.schedule();
    });
  }

  private Command runPathfindingCommand(Supplier<Pose2d> goal) {
    return new InstantCommand(() -> {
      currentPathfindingCommand.cancel();
      currentPathfindingCommand = getPathfindingCommand(goal.get());
      currentPathfindingCommand.schedule();
    });
  }

  public Command runReefPathfindingCommand(Supplier<Pair<Integer, LeftRight>> pairSupplier) {
    return runPathfindingCommand(() -> calculateReefPose(pairSupplier.get())).alongWith(Commands.runOnce(() -> {
      Pair<Integer, LeftRight> pair = pairSupplier.get();
      if (pair != null) {
        currentPathfindingPair = pair;
      }
    }));
  }

  public Command runCoralStationPathFindingCommand(Supplier<Pose2d> stationPosition) {
    return runPathfindingCommand(stationPosition);
  }

  public Command stopPathfindingCommand(){
    return new InstantCommand(() -> {
      currentPathfindingCommand.cancel();
    });
  }
}
