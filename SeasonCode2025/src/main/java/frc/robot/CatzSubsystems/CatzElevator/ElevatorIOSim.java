package frc.robot.CatzSubsystems.CatzElevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO{
    private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60Foc(2);
  private final TalonFX simTalonFX = new TalonFX(ElevatorConstants.LEFT_LEADER_ID);
  private double targetRotations;

  private final DCMotorSim m_motorSimModel = new DCMotorSim(
   LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60Foc(1), 0.001, ElevatorConstants.FINAL_RATIO
   ),
   DCMotor.getKrakenX60Foc(1)
  );

  private PIDController simPidController = new PIDController(0.006, 0.0, 0.0);
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.FINAL_RATIO,
          8.0, // mass in kg (random number)
          0.05, // drum radius in meters (random number)
          Units.inchesToMeters(ElevatorConstants.MIN_TRAVEL_INCHES),
          Units.inchesToMeters(ElevatorConstants.MAX_TRAVEL_INCHES),
          true,
          0,
          0.01,
          0.0);
  // private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  // private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("elevator root", 10, 0);
  // private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d("Elevator", 10, 90));
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1, 1);
  private final LoggedMechanismRoot2d mechanismRoot = mechanism.getRoot("elevator root", 0.63, 0);
  private final LoggedMechanismLigament2d mechanismElevator = mechanismRoot.append(new LoggedMechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double currentRotations = simTalonFX.getPosition().getValueAsDouble();
    Logger.recordOutput("Elevator/SimTargetRotations", targetRotations);
    Logger.recordOutput("Elevator/SimRotations", currentRotations);
    simTalonFX.setControl(new DutyCycleOut(simPidController.calculate(targetRotations, currentRotations)));
    var talonFXSim = simTalonFX.getSimState();
    double simCurrentSpeed = simTalonFX.get();
    m_elevatorSim.setInput(simCurrentSpeed * RobotController.getBatteryVoltage());
    Logger.recordOutput("Elevator/SimCurrentSpeed", simCurrentSpeed);

    mechanismElevator.setLength(0.5 + targetRotations * ElevatorConstants.FINAL_RATIO *0.02);
    Logger.recordOutput("Mechanism2d/Elevator", mechanism);
  }

  @Override
  public void runSetpointUp(double setpointRotations) {
      targetRotations = setpointRotations;
    
  }
}
