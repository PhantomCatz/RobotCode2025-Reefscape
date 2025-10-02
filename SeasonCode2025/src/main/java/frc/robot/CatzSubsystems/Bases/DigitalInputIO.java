package frc.robot.CatzSubsystems.Bases;

import org.littletonrobotics.junction.AutoLog;

public interface DigitalInputIO {

    @AutoLog
    public static class DigitalIOInputs {
        public boolean DigitalInput1 = false;
        public boolean DigitalInput2 = false;
    }

    public default void updateInputs(DigitalIOInputs inputs) {}

    public default DigitalIOInputs getDigitalIOInputs() {return new DigitalInputIO.DigitalIOInputs();}

}
