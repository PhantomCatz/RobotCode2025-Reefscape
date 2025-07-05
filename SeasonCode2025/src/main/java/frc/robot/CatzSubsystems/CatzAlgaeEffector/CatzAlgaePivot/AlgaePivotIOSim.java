package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;

public class AlgaePivotIOSim implements AlgaePivotIO{
    private final DCMotor m_algaePivotGearbox = DCMotor.getKrakenX60Foc(1);
    private double targetDegreesFinalShaft;
    private double elevatorHeightInch;
    private final int ALGAE_PIVOT_INDEX = 2;

    private PIDController simPidController = new PIDController(0.001, 0.0, 0.0);
    private final SingleJointedArmSim m_algaePivotSim =
    new SingleJointedArmSim(
        m_algaePivotGearbox,
        AlgaePivotConstants.ALGAE_PIVOT_GEAR_REDUCTION,
        0.025, // Used swerve drive jkg squared
        Units.inchesToMeters(16.0),
        -Math.PI,
        Math.PI, // Bounds seemed too small
        true,
        Units.degreesToRadians(AlgaePivotConstants.PIVOT_INITIAL_POS), // initial position was intially out of bounds with the math.pi/2 < 1.58
        0.01,
        0.0);

    @Override
    public void updateInputs(AlgaePivotIOInputs inputs) {
        inputs.positionDegrees = Units.radiansToDegrees(m_algaePivotSim.getAngleRads());
        inputs.velocityRpm = Units.radiansPerSecondToRotationsPerMinute(m_algaePivotSim.getVelocityRadPerSec());
        inputs.velocityRadPerSecond = m_algaePivotSim.getVelocityRadPerSec();

        double setVoltage = simPidController.calculate(inputs.positionDegrees, targetDegreesFinalShaft) * 12.0;
        m_algaePivotSim.update(0.02);
        m_algaePivotSim.setInputVoltage(setVoltage);

        Logger.recordOutput("Algae Pivot/Sim target rotations", targetDegreesFinalShaft);
        Logger.recordOutput("Algae Pivot/Sim set voltage", setVoltage);
        Logger.recordOutput("Algae Pivot/SimCurrentRPM", Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSecond));
        elevatorHeightInch = CatzElevator.Instance.getElevatorPositionInch();
        Robot.setSimPose(ALGAE_PIVOT_INDEX, new Pose3d(0.0, 0.0, Units.inchesToMeters(elevatorHeightInch), new Rotation3d(0.0, Units.degreesToRadians(inputs.positionDegrees), 0.0)));
    }

    @Override
    public void runSetpointUp(double targetDegrees, double feedforwardVolts) {
        targetDegreesFinalShaft = targetDegrees;
        // System.out.println("New algae pivot target (up): "+targetRotationsMotorShaft);
    }

    @Override
    public void runSetpointDown(double targetDegrees, double feedforwardVolts) {
        targetDegreesFinalShaft = targetDegrees;
        // System.out.println("New algae pivot target (down): "+targetRotationsMotorShaft);
    }
}
