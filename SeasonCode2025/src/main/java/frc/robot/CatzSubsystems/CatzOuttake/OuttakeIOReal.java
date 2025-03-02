// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzOuttake;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.CatzSubsystems.CatzOuttake.OuttakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
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
  private final TalonFXS IntakeCoralMtr;

  private final TalonFXSConfiguration config = new TalonFXSConfiguration();
  private SparkMaxConfig globalConfig = new SparkMaxConfig();

  public OuttakeIOReal() {

    OuttakeLeftMtr = new SparkMax(LEFT_OUTTAKE_ID, MotorType.kBrushless);
    OuttakeRightMtr = new SparkMax(RIGHT_OUTTAKE_ID, MotorType.kBrushless);
    IntakeCoralMtr = new TalonFXS(INTAKE_CORAL_ID);

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
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

    inputs.leftAppliedVolts = OuttakeLeftMtr.getBusVoltage();
    inputs.rightAppliedVolts = OuttakeRightMtr.getBusVoltage();

    inputs.velocityRpmLeft = OuttakeLeftMtr.getEncoder().getVelocity();
    inputs.velocityRpmRight = OuttakeRightMtr.getEncoder().getVelocity();

    inputs.leftCurrentAmps = OuttakeLeftMtr.getOutputCurrent();
    inputs.rightCurrentAmps = OuttakeRightMtr.getOutputCurrent();
  }

  @Override
  public void runMotor(double speed, double speed2) {
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
