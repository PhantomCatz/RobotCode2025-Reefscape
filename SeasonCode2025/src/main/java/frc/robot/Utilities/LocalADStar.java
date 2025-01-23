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
  private static final double SMOOTHING_ANCHOR_PCT = 0.8;
  private static final double EPS = 2.5;
  private static final GridPosition[] ADJACENT =
      new GridPosition[] {
        new GridPosition(-1, 0),
        new GridPosition(1, 0),
        new GridPosition(0, -1),
        new GridPosition(0, 1)
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

        System.out.println(grid.size());

        nodesY = grid.size();
        for (int row = 0; row < grid.size(); row++) {
          JSONArray rowArray = (JSONArray) grid.get(row);
          if (row == 0) {
            nodesX = rowArray.size();
          }
          for (int col = 0; col < rowArray.size(); col++) {
            boolean isObstacle = (boolean) rowArray.get(col);
            if (isObstacle) {
              System.out.print("#");
              staticObstacles.add(new GridPosition(col, row));
            } else {
              System.out.print("_");
            }
          }
          System.out.println();
        }

        JSONObject fieldSize = (JSONObject) json.get("field_size");
        fieldLength = ((Number) fieldSize.get("x")).doubleValue();
        fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
      } catch (Exception e) {
        // Do nothing, use defaults
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

  private List<GridPosition> findReversePath(
      GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
    if (sGoal.equals(sStart)) {
      return new ArrayList<>();
    }

    int iterations = 0;
    Queue<GridPosition> frontier = new LinkedList<>();
    HashMap<GridPosition, GridPosition> parent = new HashMap<>();
    frontier.add(sStart);
    parent.put(sStart, sStart);

    while (frontier.size() > 0) {
      iterations += 1;
      GridPosition currentPos = frontier.remove();
      if (currentPos.compareTo(sGoal) == 0 || iterations > 1e4) {
        break;
      }

      for (GridPosition dr : ADJACENT) {
        GridPosition newPos = currentPos.add(dr);
        if (parent.get(newPos) == null && !obstacles.contains(newPos)) {
          frontier.add(newPos);
          parent.put(newPos, currentPos);
        }
      }
    }

    List<GridPosition> retrace = new ArrayList<>();
    GridPosition currentPos = sGoal;
    while (currentPos != sStart && currentPos != null) {
      retrace.add(currentPos);
      currentPos = parent.get(currentPos);
    }
    retrace.add(sStart);

    return retrace;
  }

  private List<Waypoint> createWaypoints(
      List<GridPosition> path,
      Translation2d realStartPos,
      Translation2d realGoalPos,
      Set<GridPosition> obstacles) {
    if (path.isEmpty()) {
      return new ArrayList<>();
    }

    List<GridPosition> simplifiedPath = new ArrayList<>();
    simplifiedPath.add(path.get(path.size() - 1));
    for (int i = path.size() - 1; i > 0; i--) { //this cuts off too much for some reason, so the robot sometimes clips into the reef
      for (int j = i - 1; j > 0; j--) {
        if (!walkable(path.get(i), path.get(j), obstacles)) {
          i = j + 1;
          simplifiedPath.add(path.get(j));
          break;
        }
      }
    }
    simplifiedPath.add(path.get(0));

    List<Translation2d> fieldPosPath = new ArrayList<>();
    for (GridPosition pos : simplifiedPath) {
      fieldPosPath.add(gridPosToTranslation2d(pos));
    }

    // Replace start and end positions with their real positions
    fieldPosPath.set(0, realStartPos);
    fieldPosPath.set(fieldPosPath.size() - 1, realGoalPos);

    List<Pose2d> pathPoses = new ArrayList<>();
    pathPoses.add(
        new Pose2d(fieldPosPath.get(0), fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));
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
            fieldPosPath
                .get(fieldPosPath.size() - 1)
                .minus(fieldPosPath.get(fieldPosPath.size() - 2))
                .getAngle()));

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

  private boolean walkable(GridPosition s1, GridPosition s2, Set<GridPosition> obstacles) {
    Translation2d slope = new Translation2d(s2.x - s1.x, s2.y - s1.y);
    double n = 1 + Math.max(Math.abs(slope.getX()), Math.abs(slope.getY()));

    if (slope.getX() == 0 || slope.getY() == 0) {
      slope = slope.div(slope.getNorm());
    } else {
      slope = slope.div(Math.max(slope.getX(), slope.getY()));
    }

    double x = s1.x;
    double y = s1.y;

    while (n >= 0) {
      x += slope.getX();
      y += slope.getY();

      int gx = (int) Math.round(x);
      int gy = (int) Math.round(y);

      if (obstacles.contains(new GridPosition(gx, gy))) {
        return false;
      }
      if (gx == s2.x && gy == s2.y) {
        break;
      }
      n--;
    }

    return true;
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

    @Override
    public String toString() {
      return "(" + x + "," + y + ")";
    }
  }
}
