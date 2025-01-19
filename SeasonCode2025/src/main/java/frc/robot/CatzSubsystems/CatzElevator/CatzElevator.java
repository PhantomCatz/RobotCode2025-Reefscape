package frc.robot.CatzSubsystems.CatzElevator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.CatzElevator.ElevatorIOInputsAutoLogged;
import frc.robot.Utilities.LoggedTunableNumber;
public class CatzElevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static LoggedTunableNumber tunableNumber = new LoggedTunableNumber("Elevator/MotorPower", 0.1);   

    private double elevatorSpeed = 0.0;

    public CatzElevator() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                io = new ElevatorIOReal();
                break;
            case REPLAY:
                io = new ElevatorIOReal() {};
                break;
            default:
                io = null;
                break;
        } 
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("elevator", inputs);

        io.runMotor(elevatorSpeed);        // System.out.println(tunableNumber.get());
    }

    public Command runMotor() {
        return startEnd(() -> elevatorSpeed = tunableNumber.getAsDouble(), () -> elevatorSpeed = 0);
    }

    public Command runMotorBck() {
        return startEnd(() -> elevatorSpeed = tunableNumber.getAsDouble(), () -> elevatorSpeed = 0);
    }


    
}
