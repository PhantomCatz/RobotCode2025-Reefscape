// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzLEDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzSuperstructure.CoralState;
import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

public class CatzLED extends VirtualSubsystem {
  private static CatzLED instance = null;

  public static CatzLED getInstance() {
    if (instance == null) {
      instance = new CatzLED();
    }
    return instance;
  }

  // ----------------------------------------------------------------------------------------------
  // Robot state LED tracking
  // ----------------------------------------------------------------------------------------------
  @Getter @Setter @AutoLogOutput (key = "CatzLED/ledState")
  public ControllerLEDState ControllerState = ControllerLEDState.nuhthing;

  @Getter @Setter @AutoLogOutput (key = "CatzLED/QueueState")
  public QueueLEDState queueLEDState = QueueLEDState.EMPTY;

  @Getter @Setter @AutoLogOutput (key = "CatzLED/ledScoringState")
  public LocationState CrossbarState = LocationState.nuhthing;

  public enum QueueLEDState {
    EMPTY,
    ONE_CORAL,
    TWO_CORAL,
    THREE_CORAL,
    FOUR_CORAL
  }

  public enum ControllerLEDState {
    FULL_MANUAL,
    AQUA,
    AQUA_CLEARED,
    NBA,
    BALLS,
    CLIMB,
    REMOVE_ALGAE,
    endgameAlert,
    sameBattery,
    autoFinished,
    lowBatteryAlert,
    ledChecked,
    nuhthing
  }

  public enum LocationState {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    nuhthing
  }

  public double autoFinishedTime = 0.0;
  // MISC

  public int loopCycleCount = 0;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kPurple;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // LED PWM IDs
  private final int LEADER_LED_PWM_PORT = 1;

  // Constants
  private static final boolean paradeLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 56;
  //22 11 24
  private static final int LED_Sidebar_Start_LT = 0;
  private static final int LED_Sidebar_End_LT   = 22;
  private static final int LED_Crossbar_Start   = 23;
  private static final int LED_Crossbar_End     = 33;
  private static final int LED_Sidebar_Start_RT = 34;
  private static final int LED_Sidebar_End_RT   = 56;

  //LED Build up constants
  private static final int LED_Sidebar_Build_LT_ONE_START   = 3;
  private static final int LED_Sidebar_Build_LT_ONE_END     = 6;
  private static final int LED_Sidebar_Build_LT_TWO_START   = 7;
  private static final int LED_Sidebar_Build_LT_TWO_END     = 10;
  private static final int LED_Sidebar_Build_LT_THREE_START = 11;
  private static final int LED_Sidebar_Build_LT_THREE_END   = 14;
  private static final int LED_Sidebar_Build_LT_FOUR_START  = 15;
  private static final int LED_Sidebar_Build_LT_FOUR_END    = 18;
  private static final int LED_Sidebar_Build_RT_ONE_START   = 54;
  private static final int LED_Sidebar_Build_RT_ONE_END     = 51;
  private static final int LED_Sidebar_Build_RT_TWO_START   = 50;
  private static final int LED_Sidebar_Build_RT_TWO_END     = 47;
  private static final int LED_Sidebar_Build_RT_THREE_START = 46;
  private static final int LED_Sidebar_Build_RT_THREE_END   = 43;
  private static final int LED_Sidebar_Build_RT_FOUR_START  = 42;
  private static final int LED_Sidebar_Build_RT_FOUR_END    = 39;


  private static final double strobeDuration = 0.1;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal



  private CatzLED() {
    ledStrip = new AddressableLED(LEADER_LED_PWM_PORT);
    buffer =
        new AddressableLEDBuffer(length); // NOTE -WPILIB doesn't support creation of 2 led objects
    ledStrip.setLength(length);
    ledStrip.setData(buffer);
    ledStrip.start();

    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(Color.kBlack, Color.kWhite, System.currentTimeMillis() / 1000.0);
                ledStrip.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public void periodic() {



    // Update alliance color
    if (DriverStation.isDSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(Color.kPurple);
      secondaryDisabledColor = alliance.isPresent() ? Color.kYellow : Color.kBlack;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      // autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // -----------------------------------------------------------
    // Set LED mode
    // ----------------------------------------------------------
    setSolidElevatorColor(Color.kBlack); // Default to off

    if (estopped) {
      setSolidElevatorColor(Color.kRed);
      // MODE DISABLED
    } else if (DriverStation.isDisabled()) {
      breath(Color.kAliceBlue, Color.kWhite, breathDuration);
      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);
      } else {
        // APRILTAG CHECK
        if(ControllerState == ControllerLEDState.ledChecked) {
          setSolidElevatorColor(Color.kGreen);
        } 
      }
      // MODE AUTON
    } else if (DriverStation.isAutonomous()) {
      wave(Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
      // MODE ENABLED
    } else {
      switch(ControllerState) {
        case REMOVE_ALGAE:
          breath(Color.kAqua, Color.kRed, 0.3);
        break;
        case FULL_MANUAL:
          if(CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE) {
            setSolidElevatorColor(Color.kRed);
          }
        break;
        case NBA:
          if(CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE) {
            setSolidElevatorColor(Color.kYellow);
          } else {
            strobeElevator(Color.kYellow, Color.kBlack, breathDuration);
          }
        break;
        case AQUA:
          Color aquaColorONE   = Color.kBlack;
          Color aquaColorTWO   = Color.kBlack;
          Color aquaColorTHREE = Color.kBlack;
          Color aquaColorFOUR  = Color.kBlack;

          if(queueLEDState == QueueLEDState.ONE_CORAL) {
            aquaColorONE   = Color.kRed;
            aquaColorTWO   = Color.kBlack;
            aquaColorTHREE = Color.kBlack;
            aquaColorFOUR  = Color.kBlack;
          } else if(queueLEDState == QueueLEDState.TWO_CORAL) {
            aquaColorONE   = Color.kRed;
            aquaColorTWO   = Color.kGold;
            aquaColorTHREE = Color.kBlack;
            aquaColorFOUR  = Color.kBlack;
          } else if(queueLEDState == QueueLEDState.THREE_CORAL) {
            aquaColorONE   = Color.kRed;
            aquaColorONE   = Color.kGold;
            aquaColorTHREE = Color.kBeige;
            aquaColorFOUR  = Color.kBlack;
          } else if(queueLEDState == QueueLEDState.FOUR_CORAL) {
            aquaColorONE   = Color.kRed;
            aquaColorONE   = Color.kGold;
            aquaColorTHREE = Color.kBeige;
            aquaColorFOUR  = Color.kPurple;
          } else {
            aquaColorONE = Color.kBlack;
          }

          if(CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE) {
            buildElevator(aquaColorONE, aquaColorTWO, aquaColorTHREE, aquaColorFOUR);
          } else {
            strobeElevator(Color.kAqua, Color.kBlack, breathDuration);
          }
        break;
        case BALLS:
          if(CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE) {
            setSolidElevatorColor(Color.kWhite);
          } else {
            strobeElevator(Color.kWhite, Color.kBlack, breathDuration);
          }
        break;
        case CLIMB:
          rainbowCrossbar(rainbowCycleLength, rainbowDuration);
        break;
        case sameBattery:
          setSolidElevatorColor(Color.kDarkOrange);
        break;
        case autoFinished:
          setSolidElevatorColor(CatzConstants.CatzColorConstants.PHANTOM_SAPPHIRE);
        break;
        case lowBatteryAlert:
          setSolidElevatorColor(Color.kOrange);
        break;
        default:
          break;
      }

      switch(CrossbarState) {
        case A, G:
          setSolidCrossbarColor(Color.kRed);
          break;
        case B, H:
          setSolidCrossbarColor(Color.kBlue);
          break;
        case C, I:
          setSolidCrossbarColor(Color.kRed);
          break;
        case D, J:
          setSolidCrossbarColor(Color.kBlue);
          break;
        case E, K:
          setSolidCrossbarColor(Color.kRed);
          break;
        case F, L:
          setSolidCrossbarColor(Color.kBlue);
          break;
        default:
          break;
      }
    }

    // Update LEDs
    ledStrip.setData(buffer);
  } // end of periodic()


  //-------------------------------------------------------------------------------------------------------
  //
  //    LED factory methods
  //
  //-------------------------------------------------------------------------------------------------------
  // LED SOLID
  private void setSolidElevatorColor(Color color) {
    if (color != null) {
      for (int i = LED_Sidebar_Start_LT; i < LED_Sidebar_End_RT; i++) {
        if(!(LED_Sidebar_End_LT<i && i<LED_Sidebar_Start_RT)) {
          buffer.setLED(i, color);
        }
      }
    }
  }

  private void setSolidCrossbarColor(Color color) {
    if (color != null) {
      for (int i = LED_Crossbar_Start; i < LED_Crossbar_End; i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solid(Color color) {
    solid(1, color);
  }

  private void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
      buffer.setLED(i, color);
    }
  }


  // LED STROBE
  private void strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(c1On ? c1 : c2);
  }

  private void strobe(Color color, double duration) {
    strobe(color, Color.kBlack, duration);
  }

  private void strobeElevator(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    setSolidElevatorColor(c1On ? c1 : c2);
  }

  private void strobeCrossbar(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    setSolidCrossbarColor(c1On ? c1 : c2);
  }

  //LED Build
  private void buildElevator(Color c1, Color c2, Color c3, Color c4) {
    // BUILD ONE
    if (c1 != null) {
      for (int i = LED_Sidebar_Build_LT_ONE_START; i <= LED_Sidebar_Build_LT_ONE_END; i++) {
        if(!(LED_Sidebar_Build_LT_ONE_START < i && i < LED_Sidebar_Build_LT_ONE_END)) {
          buffer.setLED(i, c1);
        }
      }

      for (int i = LED_Sidebar_Build_RT_ONE_END; i <= LED_Sidebar_Build_RT_ONE_START; i++) {
        if(!(LED_Sidebar_Build_LT_ONE_END < i && i < LED_Sidebar_Build_LT_ONE_START)) {
          buffer.setLED(i, c1);
        }
      }
    }

    // BUILD TWO
    if (c2 != null) {
      for (int i = LED_Sidebar_Build_LT_TWO_START; i <= LED_Sidebar_Build_LT_TWO_END; i++) {
        if(!(LED_Sidebar_Build_LT_TWO_START < i && i < LED_Sidebar_Build_LT_TWO_END)) {
          buffer.setLED(i, c2);
        }
      }

      for (int i = LED_Sidebar_Build_RT_TWO_END; i <= LED_Sidebar_Build_RT_TWO_START; i++) {
        if(!(LED_Sidebar_Build_RT_TWO_END < i && i < LED_Sidebar_Build_RT_TWO_START)) {
          buffer.setLED(i, c2);
        }
      }
    }

    // BUILD THREE
    if (c3 != null) {
      for (int i = LED_Sidebar_Build_LT_THREE_START; i <= LED_Sidebar_Build_LT_THREE_END; i++) {
        if(!(LED_Sidebar_Build_LT_THREE_START < i && i < LED_Sidebar_Build_LT_THREE_END)) {
          buffer.setLED(i, c3);
        }
      }

      for (int i = LED_Sidebar_Build_RT_THREE_END; i <= LED_Sidebar_Build_RT_THREE_START; i++) {
        if(!(LED_Sidebar_Build_RT_THREE_END < i && i < LED_Sidebar_Build_RT_THREE_START)) {
          buffer.setLED(i, c3);
        }
      }
    }

    // BUILD FOUR
    if (c4 != null) {
      for (int i = LED_Sidebar_Build_LT_FOUR_START; i <= LED_Sidebar_Build_LT_FOUR_END; i++) {
        if(!(LED_Sidebar_Build_LT_FOUR_START < i && i < LED_Sidebar_Build_LT_FOUR_END)) {
          buffer.setLED(i, c4);
        }
      }

      for (int i = LED_Sidebar_Build_RT_FOUR_END; i <= LED_Sidebar_Build_RT_FOUR_START; i++) {
        if(!(LED_Sidebar_Build_RT_FOUR_END < i && i < LED_Sidebar_Build_RT_FOUR_START)) {
          buffer.setLED(i, c3);
        }
      }
    }
  }

  // LED BREATH
  private void breath(Color c1, Color c2) {
    breath(c1, c2, Timer.getFPGATimestamp());
  }

  private void breath(Color c1, Color c2, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }


  // LED Rainbow
  private void rainbowElevator(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < length; i++) {
      if(!(LED_Sidebar_End_LT<i && i<LED_Sidebar_Start_RT)) {
        x += xDiffPerLed;
        x %= 180.0;
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  private void rainbowCrossbar(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = LED_Crossbar_Start; i < LED_Crossbar_End; i++) {
        x += xDiffPerLed;
        x %= 180.0;
        buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < length; i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(List<Color> colors, int stripeLength, double duration) {
    int offset =
        (int) (Timer.getFPGATimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = 0; i < length; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }
}
