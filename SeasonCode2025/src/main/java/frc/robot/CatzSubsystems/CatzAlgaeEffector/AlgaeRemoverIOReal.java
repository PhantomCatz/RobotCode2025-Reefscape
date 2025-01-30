// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.AlgaeRemoverConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class AlgaeRemoverIOReal implements AlgaeRemoverIO {

  private final SparkMax algaeEffectorMotorTop;
  private final SparkMax algaeEffectorMotorBot;

  public AlgaeRemoverIOReal() {
    algaeEffectorMotorTop = new SparkMax(ALGAE_MOTOR_TOP_ID, MotorType.kBrushless);
    algaeEffectorMotorBot = new SparkMax(ALGAE_MOTOR_BOT_ID, MotorType.kBrushless);

    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(STALL_CURRENT_LIMIT);
    globalConfig.idleMode(IdleMode.kBrake);
    globalConfig.voltageCompensation(12);

    algaeEffectorMotorTop.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeEffectorMotorBot.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void updateInputs(AlgaeEffectorIOInputs inputs) {  }

  @Override
  public void runMotor(double speed, double speed2) {
    System.out.println(speed + " " + speed2);
    algaeEffectorMotorTop.set(-speed);
    algaeEffectorMotorBot.set(speed2);
  }

  @Override
  public void runMotorBck(double speed) {
    algaeEffectorMotorTop.set(speed);
    algaeEffectorMotorBot.set(-speed);
  }

  @Override
  public void runMotorTop(double speed) {
    algaeEffectorMotorTop.set(-speed);
  }

  @Override
  public void runMotorBottom(double speed) {
    algaeEffectorMotorBot.set(speed);
  }
}
