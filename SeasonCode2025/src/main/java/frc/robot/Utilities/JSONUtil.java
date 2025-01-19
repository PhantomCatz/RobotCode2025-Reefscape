// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Utilities;

import java.util.ArrayList;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

/** Add your docs here. */
public class JSONUtil {
  public static ArrayList<Object> getCommandsFromPath(JSONObject json) {
    return JSONUtil.getCommandsFromCommandGroup((Object) json.get("command"));
  }

  /**
   * This code will recursively loop through a json file so it can catch NamedCommands nested within
   * Sequential or Parallel commands.
   *
   * @param jsonCommandObject
   * @return List of NamedCommand as a JSON object
   */
  public static ArrayList<Object> getCommandsFromCommandGroup(Object jsonCommandObject) {
    ArrayList<Object> commandsList = new ArrayList<>();

    // --------------------------------------------------------
    // Iterate through the commands in the JSON object and add them to the list of commands.
    // If the command is a Sequential or Parallel, recursively loop through it again.
    // --------------------------------------------------------
    JSONArray commands =
        (JSONArray) ((JSONObject) ((JSONObject) jsonCommandObject).get("data")).get("commands");
    for (Object command : commands) {
      String commandType = JSONUtil.getCommandType(command);
      if (commandType.equals("sequential") || commandType.equals("parallel")) {
        commandsList.addAll(getCommandsFromCommandGroup(command));
      } else {
        commandsList.add(command);
      }
    }

    return commandsList;
  }

  public static String getCommandName(Object o) {
    return (String) ((JSONObject) ((JSONObject) o).get("data")).get("name");
  }

  public static String getCommandType(Object o) {
    return (String) ((JSONObject) o).get("type");
  }
}
