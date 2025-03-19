//------------------------------------------------------------------------------------
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems.CatzClimb;

import static frc.robot.CatzSubsystems.CatzClimb.ClimbConstants.*;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimbIOReal implements ClimbIO {
  TalonFX climbMotor = new TalonFX(CLIMB_MOTOR_ID);

  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbSupplyCurrent;
  private final StatusSignal<Current> climbTorqueCurrent;
  private final StatusSignal<Temperature> climbTempCelsius;

  public ClimbIOReal() {
    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbSupplyCurrent = climbMotor.getSupplyCurrent();
    climbTorqueCurrent = climbMotor.getTorqueCurrent();
    climbTempCelsius = climbMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        climbPosition,
        climbVelocity,
        climbAppliedVolts,
        climbSupplyCurrent,
        climbTorqueCurrent,
        climbTempCelsius);

    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();

    config.CurrentLimits.SupplyCurrentLimit = 80.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    config.MotionMagic.MotionMagicCruiseVelocity = 100;
    config.MotionMagic.MotionMagicAcceleration = 600;
    config.MotionMagic.MotionMagicJerk = 2000;

    climbMotor.getConfigurator().apply(config, 1.0);

    climbMotor.setPosition(0);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.isPositionIOMotorConnected =
        BaseStatusSignal.refreshAll(
                climbPosition,
                climbVelocity,
                climbAppliedVolts,
                climbSupplyCurrent,
                climbTorqueCurrent,
                climbTempCelsius)
            .isOK();
    inputs.positionMechs = climbPosition.getValueAsDouble();
    inputs.velocityRpm = climbVelocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = climbAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = climbSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = climbTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius = climbTempCelsius.getValueAsDouble();
  }

  @Override
  public void setPosition(double pos) // Set the motor position in mechanism rotations
      {
    CatzClimb.position = pos;
    climbMotor.setControl(positionControl.withPosition(pos));
    // System.out.println(pos);
  }

  @Override
  public void setPower(double power) {
    System.out.println("climb set power: " + power);
    climbMotor.set(power);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
    climbMotor.getConfigurator().apply(config);
  }

  @Override
  public void runCharacterizationMotor(double input) {
    climbMotor.setControl(voltageControl.withOutput(input));
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    System.out.println("kS: " + kS + " kV: " + kV + " kA: " + kA);
    climbMotor.getConfigurator().apply(config);
  }

}
