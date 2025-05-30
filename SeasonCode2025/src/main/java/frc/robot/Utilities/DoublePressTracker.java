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
package frc.robot.Utilities;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Tracker that activates only when a button is pressed twice quickly. */
public class DoublePressTracker {
  public static final double maxLengthSecs =
      0.4; // How long after the first press does the second need to occur?

  private final Trigger trigger;
  private final Timer resetTimer = new Timer();
  private DoublePressState state = DoublePressState.IDLE;

  public static Trigger createTrigger(Trigger baseTrigger) {
    var tracker = new DoublePressTracker(baseTrigger);
    return new Trigger(tracker::get);
  }

  private DoublePressTracker(Trigger baseTrigger) {
    trigger = baseTrigger;
  }

  private boolean get() {
    boolean pressed = trigger.getAsBoolean();
    switch (state) {
      case IDLE:
        if (pressed) {
          state = DoublePressState.FIRST_PRESS;
          resetTimer.reset();
          resetTimer.start();
        }
        break;
      case FIRST_PRESS:
        if (!pressed) {
          if (resetTimer.hasElapsed(maxLengthSecs)) {
            reset();
          } else {
            state = DoublePressState.FIRST_RELEASE;
          }
        }
        break;
      case FIRST_RELEASE:
        if (pressed) {
          state = DoublePressState.SECOND_PRESS;
        } else if (resetTimer.hasElapsed(maxLengthSecs)) {
          reset();
        }
        break;
      case SECOND_PRESS:
        if (!pressed) {
          reset();
        }
    }
    return state == DoublePressState.SECOND_PRESS;
  }

  private void reset() {
    state = DoublePressState.IDLE;
    resetTimer.stop();
  }

  private enum DoublePressState {
    IDLE,
    FIRST_PRESS,
    FIRST_RELEASE,
    SECOND_PRESS
  }
}
