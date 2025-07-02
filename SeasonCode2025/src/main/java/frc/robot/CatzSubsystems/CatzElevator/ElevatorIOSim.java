//------------------------------------------------------------------------------------
// 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project. 
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems.CatzElevator;

import static frc.robot.CatzSubsystems.CatzElevator.ElevatorConstants.FINAL_RATIO;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO{
  private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60Foc(2);
  private double targetRotations;
  private double currentRotations;
  private double elevatorVelocity;
  private double elevatorPositionInches;


  // private final DCMotorSim m_motorSimModel = new DCMotorSim(
  //  LinearSystemId.createDCMotorSystem(
  //     DCMotor.getKrakenX60Foc(2), 0.001, ElevatorConstants.FINAL_RATIO
  //  ),
  //  DCMotor.getKrakenX60Foc(2)
  // );

  private PIDController simPidController = new PIDController(0.1, 0.0, 0.0);
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.FINAL_RATIO,
          2.0, // mass in kg (random number)
          0.01, // drum radius in meters (random number)
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

  private Pose3d stage1 = new Pose3d(0.63, 0, 0, new Rotation3d(90, 0, 0));
  private final Pose3d[] elevatorPoses = {stage1};

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    currentRotations = elevatorPositionInches / FINAL_RATIO;
    //.recordOutput("Elevator/SimTargetRotations", targetRotations);
    //.recordOutput("Elevator/SimRotations", currentRotations);

    double setVoltage = simPidController.calculate(currentRotations, targetRotations) * 12.0;
    m_elevatorSim.setInputVoltage(setVoltage);
    m_elevatorSim.update(0.02);

    elevatorVelocity = m_elevatorSim.getVelocityMetersPerSecond();
    elevatorPositionInches = Units.metersToInches(m_elevatorSim.getPositionMeters());
    System.out.println("current elevator input " + m_elevatorSim.getInput());
    //.recordOutput("Elevator/SimCurrentSpeedMetersPerSecond", elevatorVelocity);
    //.recordOutput("Elevator/SimCurrentPositionInches", elevatorPositionInches);
    //.recordOutput("FinalComponentPoses", new Pose3d[] {new Pose3d(0.63, elevatorPositionInches / 2, 0, new Rotation3d(0.0, 90.0, 0.0))});
    mechanismElevator.setLength(0.5 + Units.inchesToMeters(elevatorPositionInches));
    //.recordOutput("Mechanism2d/Elevator", mechanism);
  }

  @Override
  public void runSetpointUp(double setpointInches) {
      double setpointRotations = setpointInches / FINAL_RATIO;
      targetRotations = setpointRotations;

  }
  @Override
  public void runSetpointDown(double setpointInches) {
      double setpointRotations = setpointInches / FINAL_RATIO;
      targetRotations = setpointRotations;

  }
}
