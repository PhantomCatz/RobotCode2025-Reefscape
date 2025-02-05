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

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;


public class OuttakeIOReal implements OuttakeIO {

  private final DigitalInput beamBreakBck;
  private final DigitalInput beamBreakFrnt;

  private final SparkMax OuttakeLeftMtr;
  private final SparkMax OuttakeRightMtr;

  private SparkMaxConfig globalConfig = new SparkMaxConfig();

  public OuttakeIOReal() {

    OuttakeLeftMtr = new SparkMax(LEFT_OUTTAKE_ID, MotorType.kBrushless);
    OuttakeRightMtr = new SparkMax(RIGHT_OUTTAKE_ID, MotorType.kBrushless);

    globalConfig.smartCurrentLimit(OUTTAKE_CURRENT_LIMIT);
    globalConfig.idleMode(IdleMode.kBrake);
    globalConfig.voltageCompensation(12);
    updateConfig();

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
  }

  @Override
  public void runMotor(double speed, double speed2) {
    OuttakeLeftMtr.set(speed);
    OuttakeRightMtr.set(-speed2);
  }

  @Override
  public void runMotorLeft(double speed) {
    OuttakeLeftMtr.set(-speed);
  }

  @Override
  public void runMotorRight(double speed) {
    OuttakeRightMtr.set(speed);
  }

  @Override
  public void setPIDF(double kP, double kI, double kD, double kF){
    globalConfig.closedLoop.pidf(kP, kI, kD, kF);
    updateConfig();
  }
}
