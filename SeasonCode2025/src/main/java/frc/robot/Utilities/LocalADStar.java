// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Utilities;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Implementation of AD* running locally in a background thread
 *
 * <p>I would like to apologize to anyone trying to understand this code. The implementation I
 * translated it from was much worse.
 */
public class LocalADStar implements Pathfinder {
  private static final double SMOOTHING_ANCHOR_PCT = 0.75; // higher values set anchors closer to waypoints
  private static final double EPS = 2.5;
  private static final GridPosition[] ADJACENT =
      new GridPosition[] {
        new GridPosition(1, 0),
        new GridPosition(0, 1),
        new GridPosition(-1, 0),
        new GridPosition(0, -1)
      };

  private double fieldLength = 16.54;
  private double fieldWidth = 8.02;

  private double nodeSize = 0.2;

  private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
  private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

  private final HashMap<GridPosition, Double> g = new HashMap<>();
  private final HashMap<GridPosition, Double> rhs = new HashMap<>();
  private final HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>();
  private final HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>();
  private final Set<GridPosition> closed = new HashSet<>();
  private final Set<GridPosition> corners = new HashSet<>();
  private final Set<GridPosition> staticObstacles = new HashSet<>();
  private final Set<GridPosition> dynamicObstacles = new HashSet<>();
  private final Set<GridPosition> requestObstacles = new HashSet<>();

  private GridPosition requestStart;
  private Translation2d requestRealStartPos;
  private GridPosition requestGoal;
  private Translation2d requestRealGoalPos;

  private double eps;

  private final Thread planningThread;
  private boolean requestMinor = true;
  private boolean requestMajor = true;
  private boolean requestReset = true;
  private boolean newPathAvailable = false;

  private final ReadWriteLock pathLock = new ReentrantReadWriteLock();
  private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

  private List<Waypoint> currentWaypoints = new ArrayList<>();
  private List<GridPosition> currentPathFull = new ArrayList<>();

  /** Create a new pathfinder that runs AD* locally in a background thread */
  public LocalADStar() {
    planningThread = new Thread(this::runThread);

    requestStart = new GridPosition(0, 0);
    requestRealStartPos = Translation2d.kZero;
    requestGoal = new GridPosition(0, 0);
    requestRealGoalPos = Translation2d.kZero;

    staticObstacles.clear();
    dynamicObstacles.clear();

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
              staticObstacles.add(new GridPosition(col, row));
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

    for(GridPosition pos: staticObstacles){
      for(int i=0; i<4; i++){
        GridPosition direction1 = ADJACENT[i];
        GridPosition direction2 = ADJACENT[(i+1)%4];

        if(
          !staticObstacles.contains(pos.add(direction1)) &&
          !staticObstacles.contains(pos.add(direction2)) &&
          !staticObstacles.contains(pos.add(direction1).add(direction2))
        ){
          corners.add(pos.add(direction1).add(direction2));
        }
      }
    }

    requestObstacles.clear();
    requestObstacles.addAll(staticObstacles);
    requestObstacles.addAll(dynamicObstacles);

    requestReset = true;
    requestMajor = true;
    requestMinor = true;

    newPathAvailable = false;

    planningThread.setDaemon(true);
    planningThread.setName("ADStar Planning Thread");
    planningThread.start();
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    return newPathAvailable;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    List<Waypoint> waypoints;

    pathLock.readLock().lock();
    waypoints = new ArrayList<>(currentWaypoints);
    pathLock.readLock().unlock();

    newPathAvailable = false;

    if (waypoints.size() < 2) {
      // Not enough points. Something got borked somewhere

      return null;
    }

    return new PathPlannerPath(waypoints, constraints, null, goalEndState);
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    GridPosition startPos = findClosestNonObstacle(getGridPos(startPosition), requestObstacles);

    if (startPos != null && !startPos.equals(requestStart)) {
      requestLock.writeLock().lock();
      requestStart = startPos;
      requestRealStartPos = startPosition;

      requestMinor = true;
      newPathAvailable = false;
      requestLock.writeLock().unlock();
    }
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), requestObstacles);

    if (gridPos != null) {
      requestLock.writeLock().lock();
      requestGoal = gridPos;
      requestRealGoalPos = goalPosition;

      requestMinor = true;
      requestMajor = true;
      requestReset = true;
      newPathAvailable = false;
      requestLock.writeLock().unlock();
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path if the robot is now within an obstacle.
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    Set<GridPosition> newObs = new HashSet<>();

    for (var obstacle : obs) {
      var gridPos1 = getGridPos(obstacle.getFirst());
      var gridPos2 = getGridPos(obstacle.getSecond());

      int minX = Math.min(gridPos1.x, gridPos2.x);
      int maxX = Math.max(gridPos1.x, gridPos2.x);

      int minY = Math.min(gridPos1.y, gridPos2.y);
      int maxY = Math.max(gridPos1.y, gridPos2.y);

      for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
          newObs.add(new GridPosition(x, y));
        }
      }
    }

    dynamicObstacles.clear();
    dynamicObstacles.addAll(newObs);
    requestLock.writeLock().lock();
    requestObstacles.clear();
    requestObstacles.addAll(staticObstacles);
    requestObstacles.addAll(dynamicObstacles);
    requestLock.writeLock().unlock();

    pathLock.readLock().lock();
    boolean recalculate = false;
    for (GridPosition pos : currentPathFull) {
      if (requestObstacles.contains(pos)) {
        recalculate = true;
        break;
      }
    }
    pathLock.readLock().unlock();

    if (recalculate) {
      setStartPosition(currentRobotPos);
      setGoalPosition(requestRealGoalPos);
    }
  }

  @SuppressWarnings("BusyWait")
  private void runThread() {
    while (true) {
      try {
        requestLock.readLock().lock();
        boolean reset = requestReset;
        boolean minor = requestMinor;
        boolean major = requestMajor;
        GridPosition start = requestStart;
        Translation2d realStart = requestRealStartPos;
        GridPosition goal = requestGoal;
        Translation2d realGoal = requestRealGoalPos;
        Set<GridPosition> obstacles = new HashSet<>(requestObstacles);

        // Change the request booleans based on what will be done this loop
        if (reset) {
          requestReset = false;
        }

        if (minor || major) {
          requestMinor = false;
          requestMajor = false;
        }
        requestLock.readLock().unlock();

        if (reset || minor || major) {
          doWork(reset, minor, major, start, goal, realStart, realGoal, obstacles);
        } else {
          try {
            Thread.sleep(10);
          } catch (InterruptedException e) {
            throw new RuntimeException(e);
          }
        }
      } catch (Exception e) {
        // Something messed up. Reset and hope for the best
        requestLock.writeLock().lock();
        requestReset = true;
        requestLock.writeLock().unlock();
      }
    }
  }

  private void doWork(
      boolean needsReset,
      boolean doMinor,
      boolean doMajor,
      GridPosition sStart,
      GridPosition sGoal,
      Translation2d realStartPos,
      Translation2d realGoalPos,
      Set<GridPosition> obstacles) {
    if (needsReset) {
      reset(sStart, sGoal);
    }

    if (doMinor || doMajor) {
      List<GridPosition> pathPositions = findReversePath(sStart, sGoal, obstacles);
      List<Waypoint> waypoints =
          createWaypoints(pathPositions, realStartPos, realGoalPos, obstacles);
      pathLock.writeLock().lock();
      currentPathFull = pathPositions;
      currentWaypoints = waypoints;
      pathLock.writeLock().unlock();

      newPathAvailable = true;
    }
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
        (o1.position.getDistance(o1.corner) + o1.cornerDistancesTraveled) -
        (o2.position.getDistance(o2.corner) + o2.cornerDistancesTraveled)
      );
    }
  }

  private List<GridPosition> findReversePath(GridPosition start, GridPosition goal, Set<GridPosition> obstacles) {
    if (goal.equals(start)) {
      return new ArrayList<>();
    }
    //Queue to store the nodes to explore, prioritizing the shortest paths.
    PriorityQueue<PathfindingPosition> frontier = new PriorityQueue<>(new CompareDistances());

    //Maps a position in the node to the corner that it is associated with.
    //Used to construct the path by retracing the corners that it traveled
    HashMap<GridPosition, GridPosition> lastCorner = new HashMap<>();

    //Invisible lines that draws a line-of-sight from corners to corners, ensuring that areas with different "lastCorners" don't spill over eachother
    HashMap<GridPosition, Set<GridPosition>> imaginaryObstacles = new HashMap<>();

    frontier.add(new PathfindingPosition(start, start, 0.0));

    //the start counts as a corner
    lastCorner.put(start, start);
    imaginaryObstacles.put(start, new HashSet<>());

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
      if(corners.contains(currentPos)){
        Translation2d slope = new Translation2d(currentPos.y - currentCorner.y, currentPos.x - currentCorner.x);
        imaginaryObstacles.get(currentCorner).addAll(getCellsOnLine(currentPos, slope, obstacles));
        imaginaryObstacles.put(currentPos, new HashSet<>());

        //add the euclidean distance from the previous corner to this corner
        currentCornerDistance += currentPos.getDistance(currentCorner);
        currentCorner = currentPos;
      }

      for(GridPosition dxy: ADJACENT){
        GridPosition newPos = currentPos.add(dxy);

        //floodfill from the current node.
        //if the next node is not an imaginary obstacle and it's not an actual obstacle and if it hasn't been traveled before, add it to the frontier.
        if(
          !imaginaryObstacles.get(currentCorner).contains(newPos) &&
          !obstacles.contains(newPos) &&
          !lastCorner.containsKey(newPos)
        ){
          frontier.add(new PathfindingPosition(newPos, currentCorner, currentCornerDistance));
          lastCorner.put(newPos, currentCorner);
        }
      }
    }

    //either a path was found (or wasn't found)
    //retrace the path backwards by going through the corners that this path visited.
    List<GridPosition> path = new ArrayList<>();
    while(goal.compareTo(start) != 0 && goal != null){
      path.add(goal);
      goal = lastCorner.get(goal);
    }

    return path;
  }

  private List<Waypoint> createWaypoints(
      List<GridPosition> path,
      Translation2d realStartPos,
      Translation2d realGoalPos,
      Set<GridPosition> obstacles) {
    if (path.isEmpty()) {
      return new ArrayList<>();
    }

    // Visualize path
    // for (int row = nodesY - 1; row >= 0; row--) {
    //   for (int col = 0; col < nodesX; col++) {
    //     if (obstacles.contains(new GridPosition(col, row))){
    //       System.out.print("#");
    //     }
    //     else if (path.contains(new GridPosition(col, row))){
    //       System.out.print("+");
    //     }
    //     else {
    //       System.out.print("_");
    //     }
    //   }
    //   System.out.println();
    // }

    List<Translation2d> fieldPosPath = new ArrayList<>();
    fieldPosPath.add(realStartPos);
    for (int i = path.size() - 1; i > 0; i--) {
      fieldPosPath.add(gridPosToTranslation2d(path.get(i)));
    }
    fieldPosPath.add(realGoalPos);

    List<Pose2d> pathPoses = new ArrayList<>();
    pathPoses.add(
      new Pose2d(
        fieldPosPath.get(0),
        fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));

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

  private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
    if (!obstacles.contains(pos)) {
      return pos;
    }

    Set<GridPosition> visited = new HashSet<>();

    Queue<GridPosition> queue = new LinkedList<>(getAllNeighbors(pos));

    while (!queue.isEmpty()) {
      GridPosition check = queue.poll();
      if (!obstacles.contains(check)) {
        return check;
      }
      visited.add(check);

      for (GridPosition neighbor : getAllNeighbors(check)) {
        if (!visited.contains(neighbor) && !queue.contains(neighbor)) {
          queue.add(neighbor);
        }
      }
    }
    return null;
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

  private void reset(GridPosition sStart, GridPosition sGoal) {
    g.clear();
    rhs.clear();
    open.clear();
    incons.clear();
    closed.clear();

    for (int x = 0; x < nodesX; x++) {
      for (int y = 0; y < nodesY; y++) {
        g.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
        rhs.put(new GridPosition(x, y), Double.POSITIVE_INFINITY);
      }
    }

    rhs.put(sGoal, 0.0);

    eps = EPS;

    open.put(sGoal, key(sGoal, sStart));
  }

  private List<GridPosition> getAllNeighbors(GridPosition s) {
    List<GridPosition> ret = new ArrayList<>();

    for (int xMove = -1; xMove <= 1; xMove++) {
      for (int yMove = -1; yMove <= 1; yMove++) {
        GridPosition sNext = new GridPosition(s.x + xMove, s.y + yMove);
        if (sNext.x >= 0 && sNext.x < nodesX && sNext.y >= 0 && sNext.y < nodesY) {
          ret.add(sNext);
        }
      }
    }
    return ret;
  }

  private Pair<Double, Double> key(GridPosition s, GridPosition sStart) {
    if (g.get(s) > rhs.get(s)) {
      return Pair.of(rhs.get(s) + eps * heuristic(sStart, s), rhs.get(s));
    } else {
      return Pair.of(g.get(s) + heuristic(sStart, s), g.get(s));
    }
  }

  private double heuristic(GridPosition sStart, GridPosition sGoal) {
    return Math.hypot(sGoal.x - sStart.x, sGoal.y - sStart.y);
  }

  private GridPosition getGridPos(Translation2d pos) {
    int x = (int) Math.round(pos.getX() / nodeSize);
    int y = (int) Math.round(pos.getY() / nodeSize);

    return new GridPosition(x, y);
  }

  private Translation2d gridPosToTranslation2d(GridPosition pos) {
    return new Translation2d(pos.x * nodeSize, pos.y * nodeSize);
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
   * @param position The current position during pathfinding.
   * @param corner   The corner that the position is associated with.
   * @param cornerDistancesTraveled The sum of distances from each corners that the position traveled through.
   */
  public record PathfindingPosition(GridPosition position, GridPosition corner, double cornerDistancesTraveled){};
}
