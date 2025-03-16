// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Utilities;

import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Implementation of AD* running locally in a background thread
 *
 * <p>I would like to apologize to anyone trying to understand this code. The implementation I
 * translated it from was much worse.
 */
public class CornerTrackingPathfinder{
  private static final double SMOOTHING_ANCHOR_PCT = 0.75; // higher values set anchors closer to waypoints
  private static final GridPosition[] ADJACENT =
      new GridPosition[] {
        new GridPosition(1, 0),
        new GridPosition(0, 1),
        new GridPosition(-1, 0),
        new GridPosition(0, -1)
      };

  private static final Translation2d[] DIAGONAL =
    new Translation2d[] {
      new Translation2d(1, 1),
      new Translation2d(-1, 1),
      new Translation2d(-1, -1),
      new Translation2d(1, -1)
    };

  private double fieldLength = 16.54;
  private double fieldWidth = 8.02;

  private double nodeSize = 0.2;

  private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
  private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

  private final Set<GridPosition> walls = new HashSet<>();
  private final HashMap<GridPosition, Integer> corners = new HashMap<>();

  /** Create a new pathfinder that runs AD* locally in a background thread */
  public CornerTrackingPathfinder() {
    File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
    if (navGridFile.exists()) {
      try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
        StringBuilder fileContentBuilder = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) {
          fileContentBuilder.append(line);
        }

        String fileContent = fileContentBuilder.toString();
        JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

        nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
        JSONArray grid = (JSONArray) json.get("grid");

        nodesY = grid.size();
        for (int row = 0; row < grid.size(); row++) {
          JSONArray rowArray = (JSONArray) grid.get(row);
          if (row == 0) {
            nodesX = rowArray.size();
          }
          for (int col = 0; col < rowArray.size(); col++) {
            boolean isObstacle = (boolean) rowArray.get(col);
            if (isObstacle) {
              walls.add(new GridPosition(col, row));
            } else {
            }
          }
        }

        JSONObject fieldSize = (JSONObject) json.get("field_size");
        fieldLength = ((Number) fieldSize.get("x")).doubleValue();
        fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
      } catch (Exception e) {
        // Do nothing, use defaults
      }
    }

    //if three squares put together in an L shape can be put around a node in the map, then the middle square of the L shape is defined as the corner
    for(GridPosition pos: walls){
      for(int i=0; i<4; i++){
        GridPosition direction1 = ADJACENT[i];
        GridPosition direction2 = ADJACENT[(i+1)%4];

        if(
          !walls.contains(pos.add(direction1)) &&
          !walls.contains(pos.add(direction2)) &&
          !walls.contains(pos.add(direction1).add(direction2))
        ){
          GridPosition corner = pos.add(direction1).add(direction2);
          if (corners.get(corner) == null){
            corners.put(corner, i);
          } else {
            corners.put(corner, -1);
          }
        }
      }
    }
  }

  public PathPlannerPath getPath(Translation2d start, Translation2d end, GoalEndState goal){
    List<Waypoint> waypoints = createWaypoints(
      findReversePath(
        findClosestNonObstacle(translation2dToGridPos(start), walls),
        findClosestNonObstacle(translation2dToGridPos(end), walls),
        walls
      ),
    start, end, walls);


    if(waypoints.size() >= 2){
      PathPlannerPath path = new PathPlannerPath(waypoints, DriveConstants.PATHFINDING_CONSTRAINTS, null, goal);
      return path;
    } else {
      return null;
    }

    // slow
    //[Waypoint[prevControl=null, anchor=Translation2d(X: 2.13, Y: 3.34), nextControl=Translation2d(X: 2.50, Y: 3.49)],
    //Waypoint[prevControl=Translation2d(X: 2.68, Y: 3.57), anchor=Translation2d(X: 3.05, Y: 3.72), nextControl=null]]

    // fast
    //[Waypoint[prevControl=null, anchor=Translation2d(X: 1.84, Y: 3.25), nextControl=Translation2d(X: 2.32, Y: 3.44)],
    //Waypoint[prevControl=Translation2d(X: 2.56, Y: 3.53), anchor=Translation2d(X: 3.05, Y: 3.72), nextControl=null]]
  }

  public PathPlannerPath getPathToNet(Translation2d start, GoalEndState goal){
    List<GridPosition> reverseNetPath = findReversePathNet(
      findClosestNonObstacle(translation2dToGridPos(start), walls),
      start.getY(),
      walls
    );

    // if(reverseNetPath.size() < 2){
    //   System.out.println("why path smol?");
    //   return null;
    // }

    List<Waypoint> waypoints = createWaypoints(
      reverseNetPath,
    start, gridPosToTranslation2d(reverseNetPath.get(0)), walls);


    if(waypoints.size() >= 2){
      PathPlannerPath path = new PathPlannerPath(waypoints, DriveConstants.PATHFINDING_CONSTRAINTS, null, goal);
      return path;
    } else {
      return null;
    }

    // slow
    //[Waypoint[prevControl=null, anchor=Translation2d(X: 2.13, Y: 3.34), nextControl=Translation2d(X: 2.50, Y: 3.49)],
    //Waypoint[prevControl=Translation2d(X: 2.68, Y: 3.57), anchor=Translation2d(X: 3.05, Y: 3.72), nextControl=null]]

    // fast
    //[Waypoint[prevControl=null, anchor=Translation2d(X: 1.84, Y: 3.25), nextControl=Translation2d(X: 2.32, Y: 3.44)],
    //Waypoint[prevControl=Translation2d(X: 2.56, Y: 3.53), anchor=Translation2d(X: 3.05, Y: 3.72), nextControl=null]]
  }


  /**
   * Comparator used in the PriorityQueue during pathfinding.
   * The queue is ordered based on the distance from the corners traveled so far and the distance from the position's associated corner.
   * This ensures that the shortest path is checked first and that the floodfilling is circular (which gurantees the shortest path and ensures that corners that were visited later doesn't flow into corners that were visited earlier).
   */
  public class CompareDistances implements Comparator<PathfindingPosition>{
    @Override
    public int compare(PathfindingPosition o1, PathfindingPosition o2) {
      return (int) Math.signum(
        (o1.position.getDistance(o1.corner) + o1.position.getDistance(o1.goal) + o1.cornerDistancesTraveled) -
        (o2.position.getDistance(o2.corner) + o2.position.getDistance(o2.goal) + o2.cornerDistancesTraveled)
      );
    }
  }


  private List<GridPosition> findReversePath(GridPosition start, GridPosition goal, Set<GridPosition> obstacles) {
    if (goal.equals(start)) {
      return new ArrayList<>();
    }
    final GridPosition g = goal;
    //Queue to store the nodes to explore, prioritizing the shortest paths.
    PriorityQueue<PathfindingPosition> frontier = new PriorityQueue<>(new CompareDistances());

    //Maps a position in the node to the corner that it is associated with.
    //Used to construct the path by retracing the corners that it traveled
    HashMap<GridPosition, GridPosition> lastCorner = new HashMap<>();

    //Invisible lines that draws a line-of-sight from corners to corners, ensuring that areas with different "lastCorners" don't spill over eachother
    HashMap<GridPosition, Set<GridPosition>> ghostWalls = new HashMap<>();

    frontier.add(new PathfindingPosition(start, start, goal, 0.0));

    //the start counts as a corner
    lastCorner.put(start, start);
    ghostWalls.put(start, new HashSet<>());

    while(frontier.size() > 0){
      PathfindingPosition currentPathfindingPos = frontier.poll();
      GridPosition currentPos = currentPathfindingPos.position;
      GridPosition currentCorner = currentPathfindingPos.corner;
      double currentCornerDistance = currentPathfindingPos.cornerDistancesTraveled;
      if(currentPos.compareTo(goal) == 0){
        break;
      }

      //if a corner is found, draw an imaginary line from the previous corner to this corner
      //now all cells propagating from this corner identifies as this corner (if that makes sense)
      if(corners.keySet().contains(currentPos)){
        Translation2d slope = new Translation2d(currentPos.y - currentCorner.y, currentPos.x - currentCorner.x);
        int diagonalIndex = corners.get(currentPos);

        //If the angle between the imaginary slope and a vector extruding diagonally from the corner (length sqrt2) is greater than 135 degrees, ignore the corner
        //The angle is measured using dot products
        if (diagonalIndex != -1 && DIAGONAL[diagonalIndex].getX() * slope.getX() + DIAGONAL[diagonalIndex].getY() * slope.getY() >= - slope.getNorm()){
          ghostWalls.get(currentCorner).addAll(getCellsOnLine(currentPos, slope, obstacles));
          ghostWalls.put(currentPos, new HashSet<>());

          //add the euclidean distance from the previous corner to this corner
          currentCornerDistance += currentPos.getDistance(currentCorner);
          currentCorner = currentPos;
        }
      }

      for(GridPosition dxy: ADJACENT){
        GridPosition newPos = currentPos.add(dxy);

        //floodfill from the current node.
        //if the next node is not an imaginary obstacle and it's not an actual obstacle and if it hasn't been traveled before, add it to the frontier.
        if(
          !ghostWalls.get(currentCorner).contains(newPos) &&
          !obstacles.contains(newPos) &&
          !lastCorner.containsKey(newPos)
        ){
          frontier.add(new PathfindingPosition(newPos, currentCorner, goal, currentCornerDistance));
          lastCorner.put(newPos, currentCorner);
        }
      }
    }


    //either a path was found (or wasn't found)
    //retrace the path backwards by going through the corners that this path visited.
    List<GridPosition> path = new ArrayList<>();
    while(goal != null && goal.compareTo(start) != 0){
      path.add(goal);
      goal = lastCorner.get(goal);
    }
    // Visualize path

    // for (int row = nodesY - 1; row >= 0; row--) {
    //   for (int col = 0; col < nodesX; col++) {
    //     if(start.equals(new GridPosition(col, row))){
    //       //starting point
    //       System.out.print("s");
    //     }
    //     else if(g.equals(new GridPosition(col, row))){
    //       System.out.print("e");
    //     }
    //     else if (obstacles.contains(new GridPosition(col, row))){
    //       //wall
    //       System.out.print("#");
    //     }
    //     else if(path.contains(new GridPosition(col, row))){
    //       //goal points
    //       System.out.print("@");
    //     }
    //     else if (lastCorner.keySet().contains(new GridPosition(col, row))){
    //       //areas flood filled
    //       System.out.print("+");
    //     }
    //     else {
    //       System.out.print("_");
    //     }
    //   }
    //   System.out.println();
    // }

    return path;
  }

  private List<GridPosition> findReversePathNet(GridPosition start, double startY, Set<GridPosition> obstacles) {
    //Queue to store the nodes to explore, prioritizing the shortest paths.
    PriorityQueue<PathfindingPosition> frontier = new PriorityQueue<>(new CompareDistances());

    //Maps a position in the node to the corner that it is associated with.
    //Used to construct the path by retracing the corners that it traveled
    HashMap<GridPosition, GridPosition> lastCorner = new HashMap<>();

    //Invisible lines that draws a line-of-sight from corners to corners, ensuring that areas with different "lastCorners" don't spill over eachother
    HashMap<GridPosition, Set<GridPosition>> ghostWalls = new HashMap<>();

    GridPosition guessGoal = translation2dToGridPos(FieldConstants.Net.getGuessClosestNetPose(startY));

    frontier.add(new PathfindingPosition(start, start, guessGoal, 0.0));

    //the start counts as a corner
    lastCorner.put(start, start);
    ghostWalls.put(start, new HashSet<>());

    while(frontier.size() > 0){
      PathfindingPosition currentPathfindingPos = frontier.poll();
      GridPosition currentPos = currentPathfindingPos.position;
      GridPosition currentCorner = currentPathfindingPos.corner;
      double currentCornerDistance = currentPathfindingPos.cornerDistancesTraveled;

      guessGoal = translation2dToGridPos(FieldConstants.Net.getGuessClosestNetPose(gridPosToTranslation2d(currentPos).getY()));
      if(currentPos.compareTo(guessGoal) == 0){
        break;
      }
      Translation2d m = new Translation2d(currentCorner.y - guessGoal.y, currentCorner.x - guessGoal.x);

      if(!hasObstacleOnLine(currentCorner, m, walls)){
        lastCorner.put(guessGoal, currentCorner);
        break;
      }

      //if a corner is found, draw an imaginary line from the previous corner to this corner
      //now all cells propagating from this corner identifies as this corner (if that makes sense)
      if(corners.keySet().contains(currentPos)){
        Translation2d slope = new Translation2d(currentPos.y - currentCorner.y, currentPos.x - currentCorner.x);
        int diagonalIndex = corners.get(currentPos);

        //If the angle between the imaginary slope and a vector extruding diagonally from the corner (length sqrt2) is greater than 135 degrees, ignore the corner
        //The angle is measured using dot products
        if (diagonalIndex != -1 && DIAGONAL[diagonalIndex].getX() * slope.getX() + DIAGONAL[diagonalIndex].getY() * slope.getY() >= - slope.getNorm()){
          ghostWalls.get(currentCorner).addAll(getCellsOnLine(currentPos, slope, obstacles));
          ghostWalls.put(currentPos, new HashSet<>());

          //add the euclidean distance from the previous corner to this corner
          currentCornerDistance += currentPos.getDistance(currentCorner);
          currentCorner = currentPos;
        }
      }

      for(GridPosition dxy: ADJACENT){
        GridPosition newPos = currentPos.add(dxy);

        //floodfill from the current node.
        //if the next node is not an imaginary obstacle and it's not an actual obstacle and if it hasn't been traveled before, add it to the frontier.
        if(
          !ghostWalls.get(currentCorner).contains(newPos) &&
          !obstacles.contains(newPos) &&
          !lastCorner.containsKey(newPos)
        ){
          frontier.add(new PathfindingPosition(newPos, currentCorner, guessGoal, currentCornerDistance));
          lastCorner.put(newPos, currentCorner);
        }
      }
    }

    //at this point the guessGoal is no longer a guess.
    final GridPosition g = guessGoal;

    //either a path was found (or wasn't found)
    //retrace the path backwards by going through the corners that this path visited.
    List<GridPosition> path = new ArrayList<>();
    while(guessGoal != null && guessGoal.compareTo(start) != 0){
      path.add(guessGoal);
      guessGoal = lastCorner.get(guessGoal);
    }
    // Visualize path

    // for (int row = nodesY - 1; row >= 0; row--) {
    //   for (int col = 0; col < nodesX; col++) {
    //     if(start.equals(new GridPosition(col, row))){
    //       //starting point
    //       System.out.print("s");
    //     }
    //     else if(g.equals(new GridPosition(col, row))){
    //       System.out.print("e");
    //     }
    //     else if (obstacles.contains(new GridPosition(col, row))){
    //       //wall
    //       System.out.print("#");
    //     }
    //     else if(path.contains(new GridPosition(col, row))){
    //       //goal points
    //       System.out.print("@");
    //     }
    //     else if (lastCorner.keySet().contains(new GridPosition(col, row))){
    //       //areas flood filled
    //       System.out.print("+");
    //     }
    //     else {
    //       System.out.print("_");
    //     }
    //   }
    //   System.out.println();
    // }

    return path;
  }

  private List<Waypoint> createWaypoints(
      List<GridPosition> path,
      Translation2d realStartPos,
      Translation2d realGoalPos,
      Set<GridPosition> obstacles) {

    List<Translation2d> fieldPosPath = new ArrayList<>();
    fieldPosPath.add(realStartPos);

    //if the path is empty, just go straight to the goal, because it means the start and goal were too close.
    if(!path.isEmpty()){
      for (int i = path.size() - 1; i > 0; i--) {
        fieldPosPath.add(gridPosToTranslation2d(path.get(i)));
      }
    }

    fieldPosPath.add(realGoalPos);

    List<Pose2d> pathPoses = new ArrayList<>();

    pathPoses.add(
      new Pose2d(
        fieldPosPath.get(0),
        fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));

    if(path.isEmpty()){
      pathPoses.add(
        new Pose2d(
          fieldPosPath.get(1),
          fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()
        )
      );
      return PathPlannerPath.waypointsFromPoses(pathPoses);
    }

    //smoothens the path by splitting a path into smaller sections at some midpoint.
    for (int i = 1; i < fieldPosPath.size() - 1; i++) {
      Translation2d last = fieldPosPath.get(i - 1);
      Translation2d current = fieldPosPath.get(i);
      Translation2d next = fieldPosPath.get(i + 1);

      Translation2d anchor1 = current.minus(last).times(SMOOTHING_ANCHOR_PCT).plus(last);
      Rotation2d heading1 = current.minus(last).getAngle();
      Translation2d anchor2 = current.minus(next).times(SMOOTHING_ANCHOR_PCT).plus(next);
      Rotation2d heading2 = next.minus(anchor2).getAngle();

      pathPoses.add(new Pose2d(anchor1, heading1));
      pathPoses.add(new Pose2d(anchor2, heading2));
    }

    pathPoses.add(
        new Pose2d(
            fieldPosPath.get(fieldPosPath.size() - 1),
            fieldPosPath.get(fieldPosPath.size() - 1).minus(fieldPosPath.get(fieldPosPath.size() - 2)).getAngle()));

    return PathPlannerPath.waypointsFromPoses(pathPoses);
  }

  // wont include start cell
  private List<GridPosition> getCellsOnLine(GridPosition start, Translation2d slope, Set<GridPosition> obstacles) {
    double n = Math.max(nodesX, nodesY);

    if (slope.getX() == 0 || slope.getY() == 0) {
      slope = slope.div(slope.getNorm());
    } else {
      slope = slope.div(Math.max(slope.getX(), slope.getY()));
    }

    double x = start.x;
    double y = start.y;
    List<GridPosition> onLine = new ArrayList<>();

    while (n >= 0) {
      x += slope.getX();
      y += slope.getY();

      int gx = (int) Math.round(x);
      int gy = (int) Math.round(y);
      if (obstacles.contains(new GridPosition(gx, gy))) {
        return onLine;
      }

      onLine.add(new GridPosition(gx, gy));
      n--;
    }

    return onLine;
  }

  private boolean hasObstacleOnLine(GridPosition start, Translation2d slope, Set<GridPosition> obstacles){
    double n = Math.max(nodesX, nodesY);

    if (slope.getX() == 0 || slope.getY() == 0) {
      slope = slope.div(slope.getNorm());
    } else {
      slope = slope.div(Math.max(slope.getX(), slope.getY()));
    }

    double x = start.x;
    double y = start.y;

    while (n >= 0) {
      x += slope.getX();
      y += slope.getY();

      int gx = (int) Math.round(x);
      int gy = (int) Math.round(y);

      if (obstacles.contains(new GridPosition(gx, gy))) {
        return true;
      }
      n--;
    }
    return false;
  }

  private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
    if (!obstacles.contains(pos)) {
      return pos;
    }

    Set<GridPosition> visited = new HashSet<>();
    Queue<GridPosition> frontier = new LinkedList<>();
    frontier.add(pos);

    while (!frontier.isEmpty()) {
      GridPosition currentPos = frontier.poll();
      if (!obstacles.contains(currentPos) && currentPos.x <= nodesX-1  && currentPos.x >= 0 && currentPos.y <= nodesY-1 && currentPos.y >= 0) {
        return currentPos;
      }
      visited.add(currentPos);

      for(GridPosition dxy: ADJACENT){
        GridPosition newPos = currentPos.add(dxy);
        if (!visited.contains(newPos)) {
          frontier.add(newPos);
        }
      }
    }
    return null;
  }

  private Translation2d gridPosToTranslation2d(GridPosition pos) {
    return new Translation2d(pos.x * nodeSize, pos.y * nodeSize);
  }

  private GridPosition translation2dToGridPos(Translation2d pos) {
    int x = (int) Math.round(pos.getX() / nodeSize);
    int y = (int) Math.round(pos.getY() / nodeSize);

    return new GridPosition(x, y);
  }

  /**
   * Represents a node in the pathfinding grid
   *
   * @param x X index in the grid
   * @param y Y index in the grid
   */
  public record GridPosition(int x, int y) implements Comparable<GridPosition> {
    @Override
    public int compareTo(GridPosition o) {
      if (x == o.x) {
        return Integer.compare(y, o.y);
      } else {
        return Integer.compare(x, o.x);
      }
    }

    public GridPosition add(GridPosition o) {
      return new GridPosition(o.x + x, o.y + y);
    }

    public double getDistance(GridPosition o){
      return Math.hypot(o.x - x, o.y - y);
    }

    @Override
    public String toString() {
      return "(" + x + "," + y + ")";
    }
  }

  /**
   * @param InitialPosition The current position during pathfinding.
   * @param corner   The corner that the position is associated with.
   * @param goal goal
   * @param cornerDistancesTraveled The sum of distances from each corners that the position traveled through.
   */
  public record PathfindingPosition(GridPosition position, GridPosition corner, GridPosition goal, double cornerDistancesTraveled){};
}
