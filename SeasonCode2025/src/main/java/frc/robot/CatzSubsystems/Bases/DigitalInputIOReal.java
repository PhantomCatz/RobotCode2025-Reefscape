package frc.robot.CatzSubsystems.Bases;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInputIOReal implements DigitalInputIO {

    DigitalInput DI1 = null;
    DigitalInput DI2 = null;

    public DigitalInputIOReal(DigitalInput input) {
        DI1 = input;
    }

    public DigitalInputIOReal(DigitalInput input1, DigitalInput input2) {
        DI1 = input1;
        DI2 = input2;
    }

    public void updateInputs(DigitalIOInputs inputs) {
        inputs.DigitalInput1 = DI1.get();
        inputs.DigitalInput2 = DI2.get();
    }

    public DigitalIOInputs getDigitalIOInputs() {
        return new DigitalInputIO.DigitalIOInputs();
    }
}
