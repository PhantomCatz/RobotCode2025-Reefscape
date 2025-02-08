// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.Reef;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.CatzSubsystems.CatzSuperstructure.LeftRight;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;

public class TeleopPosSelector {
  private final RobotContainer m_container;

  private final CommandXboxController xboxAux;
  private final String REEFSIDE = "Reefside";
  private final double SELECTION_THRESHOLD = 0.3;

  private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
  private Command currentPathfindingCommand = new InstantCommand();
  private Pair<Integer, LeftRight> currentPathfindingPair = new Pair<Integer, LeftRight>(0, LeftRight.LEFT);
  private Queue<Pair<Integer, LeftRight>> queuedPaths = new LinkedList<>();

  public TeleopPosSelector(CommandXboxController aux, RobotContainer container){
      this.xboxAux = aux;
      this.m_container = container;

      for(int i = 0; i < 6; i++){
          SmartDashboard.putBoolean(REEFSIDE + i, false);
      }
  }

  public void pathQueueAdd(Pair<Integer, LeftRight> pose){
    queuedPaths.add(pose);
  }
  public Pair<Integer, LeftRight> pathQueuePeek(){
    return queuedPaths.peek();
  }
  public Pair<Integer, LeftRight> pathQueuePop(){
    return queuedPaths.poll();
  }
  public void pathQueueClear(){
    queuedPaths.clear();
  }

  public void updateCurrentlySelected(){
    Pair<Integer, LeftRight> currentlySelected = getXBoxReefPos();
    if(currentlySelected.getFirst() == -1){
      currentlySelected = getClosestReefPos();
    }

    for(int side = 0; side < 6; side++){
      SmartDashboard.putBoolean(REEFSIDE + side, side == currentlySelected.getFirst());
    }
  }

  public Pair<Integer, LeftRight> getXBoxReefPos(){
    final double x = -xboxAux.getLeftY();
    final double y = -xboxAux.getLeftX();

    if(Math.hypot(x, y) > SELECTION_THRESHOLD){
        //ensures angle is between 0-2pi (Dr. Eric Yuchen Lu (MD)'s idea)
        double angle = (Math.atan2(y, x) + 2*Math.PI) % (2 * Math.PI);

        //if angle is too close to 2pi, then it will return 6, but we want selected to be between 0-5 (Dr. Eric Yuchen Lu (MD)'s idea)
        int side = (int) Math.round(angle * 3.0 / Math.PI) % 6;

        LeftRight leftRight = LeftRight.LEFT;
        if (xboxAux.rightBumper().getAsBoolean()){
          leftRight = LeftRight.RIGHT;
        }

        return new Pair<Integer, LeftRight>(side, leftRight);
    }else{
        return new Pair<Integer, LeftRight>(-1, LeftRight.LEFT);
    }
  }

  public Pair<Integer, LeftRight> getClosestReefPos(){
    int closestSide = 0;
    LeftRight closestLeftRight = LeftRight.LEFT;
    Translation2d robotPos = tracker.getEstimatedPose().getTranslation();

    for(int side = 0; side < 6; side++){
      for(LeftRight leftRight: LeftRight.values()){
        Pose2d reefPos = calculateReefPose(side, leftRight);
        if(reefPos.getTranslation().getDistance(robotPos) < calculateReefPose(closestSide, closestLeftRight).getTranslation().getDistance(robotPos)){
          closestSide = side;
          closestLeftRight = leftRight;
        }
      }
    }

    return new Pair<Integer, LeftRight>(closestSide, closestLeftRight);
  }

  public Pose2d calculateReefPose(int reefAngle, LeftRight leftRightPos){
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

  public Pose2d calculateReefPose(Pair<Integer, LeftRight> pair){
    if(pair == null){
      return null;
    } else {
      return calculateReefPose(pair.getFirst(), pair.getSecond());
    }
  }

  public Command getPathfindingCommand(Pose2d goal) {
    if (goal == null){
      return new InstantCommand();
    }

    Translation2d robotPos = tracker.getEstimatedPose().getTranslation();
    Pathfinding.setStartPosition(robotPos);
    Pathfinding.setGoalPosition(goal.getTranslation());
    PathPlannerPath path =
        Pathfinding.getCurrentPath(
            DriveConstants.PATHFINDING_CONSTRAINTS, new GoalEndState(0, goal.getRotation()));

    if (path == null) {
      return new InstantCommand();
    } else {
      if(AllianceFlipUtil.shouldFlipToRed()) {
        path = path.flipPath();
      }
      return new TrajectoryDriveCmd(path, m_container.getCatzDrivetrain(), true);
    }
  }

  public Command runLeftRightCommand(LeftRight leftRight){
    return new InstantCommand(() -> {
      Pose2d goal = calculateReefPose(new Pair<Integer, LeftRight>(currentPathfindingPair.getFirst(), leftRight));
      Pose2d currentPose = calculateReefPose(currentPathfindingPair);
      if(currentPose.getTranslation().getDistance(goal.getTranslation()) > Reef.leftRightDistance * 3){
        return;
      }

      
    });

    currentPathfindingCommand.cancel();
      currentPathfindingCommand = new TrajectoryDriveCmd(new PathPlannerPath(
        Arrays.asList(new Waypoint[] {new Waypoint(goal), new Waypoint(goal)}),
        DriveConstants.PATHFINDING_CONSTRAINTS,
        null,
        new GoalEndState(0, goal.getRotation())
      ), m_container.getCatzDrivetrain(), true);
      currentPathfindingCommand.schedule();
  }

  private Command runPathfindingCommand(Supplier<Pose2d> goal){
    return new InstantCommand(() -> {
      currentPathfindingCommand.cancel();
      currentPathfindingCommand = getPathfindingCommand(goal.get());
      currentPathfindingCommand.schedule();
    });
  }

  public Command runReefPathfindingCommand(Supplier<Pair<Integer, LeftRight>> pairSupplier){
    return runPathfindingCommand(() -> calculateReefPose(pairSupplier.get())).alongWith(Commands.runOnce(() -> {
      Pair<Integer, LeftRight> pair = pairSupplier.get();
      if (pair != null){
        currentPathfindingPair = pair;
      }
    }));
  }

  public Command stopPathfindingCommand(){
    return new InstantCommand(() -> {
      currentPathfindingCommand.cancel();
    });
  }
}
