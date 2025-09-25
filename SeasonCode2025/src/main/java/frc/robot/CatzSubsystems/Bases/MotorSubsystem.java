package frc.robot.CatzSubsystems.Bases;

import frc.robot.CatzSubsystems.Bases.*;
import frc.robot.CatzSubsystems.Bases.MotorIO.Setpoint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class MotorSubsystem<IO extends MotorIO> extends SubsystemBase {
    protected final IO io;
	protected final String name;

    public MotorSubsystem(IO io, String name) {
		super(name);
		this.io = io;
		this.name = name;
	}

    @Override
	public void periodic() {
		io.updateInputs(io.getMotorIOInputs());
	}

    public void applySetpoint(Setpoint setpoint) {
		io.applySetpoint(setpoint);
	}
}
