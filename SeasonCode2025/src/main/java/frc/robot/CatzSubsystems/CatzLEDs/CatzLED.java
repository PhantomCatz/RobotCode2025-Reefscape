
package frc.robot.CatzSubsystems.CatzLEDs;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants.CatzColorConstants;

public class CatzLED extends SubsystemBase {
    private static CatzLED instance = null;
    
    public static CatzLED getInstance() {
        if(instance == null) {
            instance = new CatzLED();
        }
        return instance;
    }

    //----------------------------------------------------------------------------------------------
    // Robot state LED tracking
    //----------------------------------------------------------------------------------------------
    // MISC
    public int loopCycleCount = 0;
    public boolean boost = false;
    public boolean intaking = false;
    public boolean hasNoteSpeaker = false;
    public boolean hasNoteAmp      = false;

    // Main Driver LED States
    public boolean isAutoShootSpeaker = false;
    public boolean isAmping = false;
    public boolean isHoarding = false;
    public boolean isClimbing = false;

    
    // Extra Driver LED States
    public boolean trapping = false;
    public boolean autoDrive = false;

    // MISC LED states
    public boolean endgameAlert = false;
    public boolean sameBattery = false;
    public boolean autoFinished = false;
    public double autoFinishedTime = 0.0;
    public boolean lowBatteryAlert = false;
    public boolean demoMode = false;

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
    private final int LEADER_LED_PWM_PORT = 2; 
    private final int LED_COUNT_HALF = 5; //TODO what does this mean


    // Constants
    private static final boolean paradeLeds = false;
    private static final int minLoopCycleCount = 10;
    private static final int length = 34;
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
        buffer = new AddressableLEDBuffer(length); // NOTE -WPILIB doesn't support creation of 2 led objects
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
    public void periodic(){
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
            autoFinished = false;
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

        //-----------------------------------------------------------
        // Set LED mode
        //----------------------------------------------------------
        solid(Color.kBlack); // Default to off
        if (estopped) {
            solid(Color.kRed);
        // MODE DISABLED
        } else if (DriverStation.isDisabled()) {
            if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
                // Auto fade
                solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);
            } else if (lowBatteryAlert) {
                // Low battery
                solid(Color.kOrangeRed);
            } else if (paradeLeds) {
                // TODO add parade led pattern
            } else {
                // Default pattern
                wave(allianceColor, secondaryDisabledColor, waveAllianceCycleLength, waveAllianceDuration);
            }

            // Same battery alert //TODO add battery alert
            if (sameBattery) {
                breath(Color.kRed, Color.kBlack);
            }

        // MODE AUTON
        } else if (DriverStation.isAutonomous()) {
            wave(Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
            if (autoFinished) {
                double fullTime = (double) length / waveFastCycleLength * waveFastDuration;
                solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
            }
        // MODE ENABLED
        } else { 
            wave(CatzColorConstants.PHANTOM_SAPPHIRE, Color.kWhite, waveAllianceCycleLength, waveAllianceDuration);

            if (trapping || isClimbing || autoDrive || isAutoShootSpeaker) {
                rainbow(rainbowCycleLength, rainbowDuration);
            } else if(isHoarding) {
                wave(Color.kBlack, Color.kPurple, waveAllianceCycleLength, waveAllianceDuration);
            } else if (hasNoteSpeaker) {
                solid(Color.kGreen);
            } else if (hasNoteAmp) {
                solid(Color.kOrange);
            }

            if (endgameAlert) {
                strobe(Color.kRed, Color.kGold, strobeDuration);
            }
        }

        // Update LEDs
        ledStrip.setData(buffer);
    } //end of periodic()

    private void solid(Color color) {
        if (color != null) {
            for (int i = 0; i < length; i++) {
                buffer.setLED(i, color);
            }
        }
    }

    private void solid(double percent, Color color) {
        for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
        buffer.setLED(i, color);
        }
    }

    private void strobe(Color c1, Color c2, double duration) {
        boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(c1On ? c1 : c2);
    }

    private void strobe(Color color, double duration) {
        strobe(color, Color.kBlack, duration);
    }

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

    private void rainbow(double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
        double xDiffPerLed = 180.0 / cycleLength;
        for (int i = 0; i < length; i++) {
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