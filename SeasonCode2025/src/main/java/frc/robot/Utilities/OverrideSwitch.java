package frc.robot.Utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for physical override switches on operator console. */
public class OverrideSwitch {
  private final GenericHID joystick;

  public OverrideSwitch(int port) {
    joystick = new GenericHID(port);
  }

  /** Returns whether the controller is connected. */
  public boolean isConnected() {
    return joystick.isConnected();
  }

  /** Gets the state of an operator-side switch (0-4 from left to right). */
  public boolean getAuxSwitch(int index) {
    return joystick.getRawButton(index);
  }

  /** Gets the state of the multi-directional switch. */
  public MultiDirectionSwitchState getMultiDirectionSwitch() {
    if (joystick.getRawButton(4)) {
      return MultiDirectionSwitchState.LEFT;
    } else if (joystick.getRawButton(5)) {
      return MultiDirectionSwitchState.RIGHT;
    } else {
      return MultiDirectionSwitchState.NEUTRAL;
    }
  }

  /** Returns a trigger for an operator-side switch (0-4 from left to right). */
  public Trigger auxSwitch(int index) {
    return new Trigger(() -> getAuxSwitch(index));
  }

  /** Returns a trigger for when the multi-directional switch is pushed to the left. */
  public Trigger multiDirectionSwitchLeft() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.LEFT);
  }

  /** Returns a trigger for when the multi-directional switch is pushed to the right. */
  public Trigger multiDirectionSwitchRight() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.RIGHT);
  }

  /** The state of the multi-directional switch. */
  public static enum MultiDirectionSwitchState {
    LEFT,
    NEUTRAL,
    RIGHT
  }
}
