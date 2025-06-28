package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaePivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.CatzSubsystems.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants;

public class AlgaePivotIOSim implements AlgaePivotIO{
    private final DCMotor m_algaePivotGearbox = DCMotor.getKrakenX60Foc(1);
    private double currentRotations;
    private double targetRotations;
    private double algaePivotVelocityRadiansPerSecond;
    private double elevatorHeightInch;
    private final int ALGAE_PIVOT_INDEX = 2;
    private Pose3d[] algaePivotPose3d = new Pose3d[] {
        new Pose3d(0.0, 0.0, Units.inchesToMeters(ElevatorConstants.START_HEIGHT_GROUND), new Rotation3d(0.0, Math.PI / 2, 0.0))
    };
    
    private PIDController simPidController = new PIDController(0.1, 0.0, 0.0);
    private final SingleJointedArmSim m_algaePivotSim = 
    new SingleJointedArmSim(
        m_algaePivotGearbox, 
        AlgaePivotConstants.ALGAE_PIVOT_GEAR_REDUCTION, 
        1584.0, 
        Units.inchesToMeters(16.0), 
        -Math.PI / 2, 
        Math.PI / 2, 
        true, 
        Units.degreesToRadians(AlgaePivotConstants.PIVOT_INITIAL_POS), 
        0.01,
        0.0);

    @Override
    public void updateInputs(AlgaePivotIOInputs inputs) {
        currentRotations = Units.radiansToRotations(m_algaePivotSim.getAngleRads());
        Logger.recordOutput("Algae Pivot/Sim current rotations", currentRotations);
        Logger.recordOutput("Algae Pivot/Sim target rotations", targetRotations);

        double setVoltage = simPidController.calculate(currentRotations, targetRotations) * 12.0;
        m_algaePivotSim.setInputVoltage(setVoltage);
        m_algaePivotSim.update(0.02);

        algaePivotVelocityRadiansPerSecond = m_algaePivotSim.getVelocityRadPerSec();
        Logger.recordOutput("Algae Pivot/SimCurrentRadiansPerSecond", algaePivotVelocityRadiansPerSecond);
        elevatorHeightInch = CatzElevator.Instance.getElevatorPositionInch();
        algaePivotPose3d[0] = new Pose3d(0.0, 0.0, elevatorHeightInch, new Rotation3d(0.0, Units.radiansToRotations(currentRotations), 0.0));
    }

    @Override
    public void runSetpointUp(double targetDegrees, double feedforwardVolts) {
        targetRotations = Units.degreesToRotations(targetDegrees) * AlgaePivotConstants.ALGAE_PIVOT_GEAR_REDUCTION;
    }

    @Override
    public void runSetpointDown(double targetDegrees, double feedforwardVolts) {
        targetRotations = Units.degreesToRotations(targetDegrees) * AlgaePivotConstants.ALGAE_PIVOT_GEAR_REDUCTION;
    }

    public Pose3d[] getAlgaePivotPose3d() {
        return algaePivotPose3d;
    }
}
