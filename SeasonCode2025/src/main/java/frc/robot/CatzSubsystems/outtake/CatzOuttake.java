package frc.robot.CatzSubsystems.outtake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.CatzConstants;


public class CatzOuttake extends SubsystemBase{

    private final OuttakeIO io;
    private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

    private static LoggedTunableNumber tunableNumber = new LoggedTunableNumber("Intake/MotorPower", 0.1);   
    
    private double outtakeSpeed = 0.0;
    
    public CatzOuttake() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                io = new OuttakeIOSparkmax();
                break;
            case REPLAY:
                io = new OuttakeIOSparkmax() {};
                break;
            default:
                io = null;
                break;
        }    
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);

        io.runMotor(outtakeSpeed);        // System.out.println(tunableNumber.get());
    }

    public Command runMotor() {
        return startEnd(() -> outtakeSpeed = tunableNumber.getAsDouble(), () -> outtakeSpeed = 0);
    }
  
}