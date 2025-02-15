// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzOuttake;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.CatzSubsystems.CatzOuttake.OuttakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;


public class OuttakeIOReal implements OuttakeIO {

  private final DigitalInput beamBreakBck;
  private final DigitalInput beamBreakFrnt;

  private final SparkMax OuttakeLeftMtr;
  private final SparkMax OuttakeRightMtr;
  private final TalonFX IntakeCoralMtr;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private SparkMaxConfig globalConfig = new SparkMaxConfig();

  public OuttakeIOReal() {

    OuttakeLeftMtr = new SparkMax(LEFT_OUTTAKE_ID, MotorType.kBrushless);
    OuttakeRightMtr = new SparkMax(RIGHT_OUTTAKE_ID, MotorType.kBrushless);
    IntakeCoralMtr = new TalonFX(INTAKE_CORAL_ID);

    // globalConfig.smartCurrentLimit(OUTTAKE_CURRENT_LIMIT);
    globalConfig.smartCurrentLimit(20);
    globalConfig.idleMode(IdleMode.kBrake);
    globalConfig.voltageCompensation(12);
    updateConfig();

    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kA = gains.kA();
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();

    config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
    // config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
    // config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    IntakeCoralMtr.setPosition(0);

    IntakeCoralMtr.getConfigurator().apply(config, 1.0);

    beamBreakBck = new DigitalInput(BACK_BEAM_BREAK_ID);
    beamBreakFrnt = new DigitalInput(FRONT_BEAM_BREAK_ID);
  }

  private void updateConfig(){
    OuttakeLeftMtr.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    OuttakeRightMtr.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.bbreakFrntTriggered = !beamBreakFrnt.get();
    inputs.bbreakBackTriggered = !beamBreakBck.get();
    inputs.appliedVolts        = OuttakeLeftMtr.getBusVoltage();
    inputs.rightAppliedVolts   = OuttakeRightMtr.getBusVoltage();
  }

  @Override
  public void runMotor(double speed, double speed2) {
    // System.out.println(speed);
    OuttakeLeftMtr.set(speed);
    OuttakeRightMtr.set(-speed2);
  }

  @Override
  public void runIntakesIntakeMotor(double speed) {
    IntakeCoralMtr.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void setPIDF(double kP, double kI, double kD, double kF){
    globalConfig.closedLoop.pidf(kP, kI, kD, kF);
    updateConfig();
  }
}
