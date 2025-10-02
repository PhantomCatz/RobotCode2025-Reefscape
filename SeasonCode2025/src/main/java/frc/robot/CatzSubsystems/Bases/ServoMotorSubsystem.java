package frc.robot.CatzSubsystems.Bases;

import frc.robot.CatzSubsystems.Bases.*;
import frc.robot.CatzSubsystems.Bases.ServoMotorIO.Setpoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoMotorSubsystem<IO extends ServoMotorIO> extends SubsystemBase {
    protected final IO io;
	protected final String name;

    public ServoMotorSubsystem(IO io, String name) {
		super(name);
		this.io = io;
		this.name = name;
	}

    @Override
	public void periodic() {
		io.updateInputs(io.getServoMotorIOInputs());
	}

	public void applySetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}

	public Command setpointCommand(Setpoint setpoint) {
		return runOnce(() -> applySetpoint(setpoint));
	}




}
