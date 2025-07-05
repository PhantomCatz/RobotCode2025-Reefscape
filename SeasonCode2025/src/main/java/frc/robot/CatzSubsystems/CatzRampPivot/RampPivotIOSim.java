package frc.robot.CatzSubsystems.CatzRampPivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class RampPivotIOSim implements RampPivotIO {
    private final DCMotor m_rampPivotGearbox = DCMotor.getKrakenX60Foc(1);
    private final int RAMP_PIVOT_INDEX = 3;
    private double targetDegreesFinalShaft;

    private PIDController simPidController = new PIDController(0.001, 0.0, 0.0);
    private final SingleJointedArmSim m_rampPivotSim =
    new SingleJointedArmSim(
        m_rampPivotGearbox,
        RampPivotConstants.GEAR_RATIO,
        0.025,
        Units.inchesToMeters(9.0),
        0,
        Math.PI*2/3,
        true,
        Math.PI*2/3,
        0.01,
        0.0);

    @Override
    public void updateInputs(RampPivotIOInputs inputs) {
        inputs.positionDegrees = Units.radiansToDegrees(m_rampPivotSim.getAngleRads());
        inputs.velocityRpm = Units.radiansPerSecondToRotationsPerMinute(m_rampPivotSim.getVelocityRadPerSec());

        double setVoltage = simPidController.calculate(inputs.positionDegrees, targetDegreesFinalShaft) * 12.0;
        m_rampPivotSim.update(0.02);
        m_rampPivotSim.setInputVoltage(setVoltage);

        Logger.recordOutput("RampPivot/Sim target rotations", targetDegreesFinalShaft);
        Logger.recordOutput("RampPivot/Sim set voltage", setVoltage);
        Logger.recordOutput("RampPivot/SimCurrentRPM", inputs.velocityRpm);
        Robot.setSimPose(RAMP_PIVOT_INDEX, new Pose3d(new Translation3d(0.0, 0.0, 0.0).plus(RampPivotConstants.RAMP_PIVOT_SIM_OFFSET), new Rotation3d(0.0, Units.degreesToRadians(inputs.positionDegrees), 0.0)));
    }

    @Override
    public void setPosition(double setpointRads, double feedforward) {
        targetDegreesFinalShaft = Units.radiansToDegrees(setpointRads);
    }
}
