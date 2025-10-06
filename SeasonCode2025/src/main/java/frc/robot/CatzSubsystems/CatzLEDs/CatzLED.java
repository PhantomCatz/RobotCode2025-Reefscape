package frc.robot.CatzSubsystems.CatzLEDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
import frc.robot.CatzSubsystems.CatzSuperstructure.CoralState;
import frc.robot.CatzSubsystems.CatzSuperstructure.Gamepiece;
import frc.robot.Utilities.VirtualSubsystem;
import lombok.Getter;
import lombok.Setter;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

public class CatzLED extends VirtualSubsystem {
  public static final CatzLED Instance = new CatzLED();

  // ----------------------------------------------------------------------------------------------
  // Robot state LED tracking
  // ----------------------------------------------------------------------------------------------
  @Getter @Setter @AutoLogOutput (key = "CatzLED/ElevatorLEDState")
  public ControllerLEDState controllerState = ControllerLEDState.nuhthing;

  @Getter @Setter @AutoLogOutput (key = "CatzLED/QueueState")
  public QueueLEDState queueLEDState = QueueLEDState.EMPTY;

  @Getter @Setter @AutoLogOutput(key = "CatzSuperstructure/isClimbExtendingOut")
  private WinchingState climbDirection = WinchingState.IDLE;

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
    lowBatteryAlert,
    ledChecked,
    nuhthing
  }

  public enum WinchingState {
    EXTENDING(Color.kGreen),
    RETRACTING(Color.kRed),
    IDLE(Color.kBlack);

    private final Color color;
    private WinchingState(Color color) {
        this.color = color;
    }
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
  private final int LEADER_LED_PWM_PORT = 0;

  // Constants
  private static final boolean paradeLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 54;
  //24 7 23
  private static final int LED_Sidebar_Start_RT = 0;
  private static final int LED_Sidebar_End_RT   = 23;
  private static final int LED_Crossbar_Start   = 24;
  private static final int LED_Crossbar_End     = 30;
  private static final int LED_Sidebar_Start_LT = 31;
  private static final int LED_Sidebar_End_LT   = 53;

  private static final double strobeDuration = 0.1;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double bubbleTime = 2.5;
  private static final double autoFadeTime = 1.3; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal
  private static final double coralThrowTime = 0.5; // tune

  private static final Color[] autonCountdownColors = {Color.kGreen, Color.kYellow, Color.kRed, Color.kWhite};

  private CatzLED() {
    ledStrip = new AddressableLED(LEADER_LED_PWM_PORT);
    buffer = new AddressableLEDBuffer(length); // NOTE -WPILIB doesn't support creation of 2 led objects
    ledStrip.setLength(length);
    ledStrip.setData(buffer);
    ledStrip.start();

    loadingNotifier = new Notifier(
                            () -> {
                              synchronized (this) {
                                breath(Color.kBlack, Color.kWhiteSmoke, System.currentTimeMillis() / 1000.0);
                                ledStrip.setData(buffer);
                              }
                            }
    );
    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public void periodic() {
    // Update alliance color
    if (DriverStation.isDSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kAqua : Color.kOrangeRed)
              .orElse(Color.kPurple);
      secondaryDisabledColor = alliance.isPresent() ? Color.kYellow : Color.kBlack;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {

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


    // -----------------------------------------------------------------------------------------------
    // Set Elevator LED state
    // -----------------------------------------------------------------------------------------------
    if (estopped) {
      setSolidElevatorColor(Color.kRed);
      // MODE DISABLED
    } else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && (Timer.getFPGATimestamp() - lastEnabledTime < bubbleTime)) {
        // Auto fade
        bubble((int) ((((Timer.getFPGATimestamp() - lastEnabledTime) % bubbleTime)) / bubbleTime * LED_Sidebar_End_RT), Color.kAqua);
      } else {
        breath(Color.kPurple, Color.kAqua, System.currentTimeMillis()/10000.0);
        bubble((int) ((((Timer.getFPGATimestamp() - lastEnabledTime) % bubbleTime)) / bubbleTime * LED_Sidebar_End_RT), allianceColor);
        // APRILTAG CHECK
        if(controllerState == ControllerLEDState.ledChecked) {
          bigBubble((int) ((Timer.getFPGATimestamp() / bubbleTime) * LED_Sidebar_End_RT), 4, 10, Color.kGreen);
        }
      }
      // MODE AUTON
    } else if (DriverStation.isAutonomous()) {
      double timeLeft = CatzRobotTracker.Instance.getCoralStationTrajectoryRemaining();
      // count down to coral drop time with in half second intervals. Drop right when it turns green. If there is more than 1 second white.
      setSolidElevatorColor(autonCountdownColors[(int) Math.max(Math.min(Math.ceil((timeLeft-coralThrowTime)/0.5), 3.0), 0.0)]);
      // MODE ENABLED
    } else {
      switch(controllerState) {
        case REMOVE_ALGAE:
          breath(Color.kAqua, Color.kRed, 0.3);
        break;
        case FULL_MANUAL:
          if(CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE) {
            bigBubble((int) ((Timer.getFPGATimestamp() / bubbleTime) * LED_Sidebar_End_RT), 4, 10, Color.kAqua);
          } else {
            strobeElevator(Color.kAqua, Color.kBlack, breathDuration);
          }
        break;
        case NBA:
          if(!CatzRobotTracker.Instance.isReachedGoal()) {
            wave(Color.kRed, Color.kBlack, waveFastCycleLength, waveFastDuration);
          } else {
            bigBubble((int) ((Timer.getFPGATimestamp() / bubbleTime) * LED_Sidebar_End_RT), 4, 10, Color.kWhite);
          }
        break;
        case AQUA:
          setWaveElevatorColor(Color.kAqua, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
        break;

        case BALLS:
          if(CatzSuperstructure.getCurrentCoralState() == CoralState.IN_OUTTAKE) {
            setSolidElevatorColor(Color.kWhite);
          } else {
            strobeElevator(Color.kWhite, Color.kBlack, breathDuration);
          }
        break;
        case CLIMB:
          rainbowElevator(rainbowCycleLength, rainbowDuration);
          setSolidCrossbarColor(climbDirection.color);
        break;
        case sameBattery:
          setSolidElevatorColor(Color.kDarkOrange);
        break;
        case lowBatteryAlert:
          setSolidElevatorColor(Color.kOrange);
        break;
        default:
          break;
        }


      //---------------------------------------------------------------------------------------------
      // Update crossbar state
      //---------------------------------------------------------------------------------------------
      if(CatzSuperstructure.getChosenGamepiece() == Gamepiece.ALGAE) {
        setSolidCrossbarColor(Color.kAqua);
      } else {
        setSolidCrossbarColor(Color.kGreen);
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
      for (int i = LED_Sidebar_Start_RT; i <= LED_Sidebar_End_LT; i++) {
        if(!(LED_Sidebar_End_RT<i && i<LED_Sidebar_Start_LT)) {
          buffer.setLED(i, color);
        }
      }
    }
  }

  private void setWaveElevatorColor(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < length; i++) {
      if(!(LED_Sidebar_End_RT<i && i<LED_Sidebar_Start_LT)) {
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
        buffer.setLED(i, new Color(red, green, blue));      }
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
    for (int i = (int) Math.ceil(MathUtil.clamp(length * percent, 0, length)); i<length; i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

  private void bigBubble(int colored, int bubbleLength, int bubbleInterval, Color color) {
    //System.out.println("big bubble" + colored);
    for (int i=0; i<LED_Crossbar_Start; i++) {
      if (i <= colored && ((i - colored % bubbleInterval) + bubbleInterval) % bubbleInterval < bubbleLength) {
        buffer.setLED(i, color);
        buffer.setLED(LED_Sidebar_End_LT-i, color);
      }
      else {
        buffer.setLED(i, Color.kBlack);
        buffer.setLED(LED_Sidebar_End_LT-i, Color.kBlack);
      }
    }
    // buffer.setLED(47, Color.kBlack);
    // buffer.setLED(48, Color.kBlack);

  }

  private void bubble(int colored, Color color) {
    // System.out.println("bubble light"+colored);
    for (int i=0; i<LED_Crossbar_Start; i++) {
      if (i <= colored && i % 3 == colored % 3) {
        buffer.setLED(i, color);
        buffer.setLED(LED_Sidebar_End_LT-i, color);
      }
      else {
        buffer.setLED(i, Color.kBlack);
        buffer.setLED(LED_Sidebar_End_LT-i, Color.kBlack);
      }
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
    //System.out.println("strobe");
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    setSolidElevatorColor(c1On ? c1 : c2);
  }

  private void strobeCrossbar(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    setSolidCrossbarColor(c1On ? c1 : c2);
  }

  //LED Build
  private void buildElevator(Color c1, Color c2, Color c3, Color c4) {

  }

  // LED BREATH
  private void breath(Color c1, Color c2) {
    breath(c1, c2, System.currentTimeMillis() / 1000.0);
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
      if(!(LED_Sidebar_End_RT<i && i< LED_Sidebar_Start_LT)) {
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
    //System.out.println("wave");
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
