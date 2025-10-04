package frc.robot.CatzSubsystems.Bases;

import frc.robot.CatzSubsystems.Bases.*;
import frc.robot.CatzSubsystems.Bases.ServoMotorIO.Setpoint;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem<IO extends MotorIOReal> extends SubsystemBase {
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
		//System.out.println("it worked!!!! base");
	}

}
