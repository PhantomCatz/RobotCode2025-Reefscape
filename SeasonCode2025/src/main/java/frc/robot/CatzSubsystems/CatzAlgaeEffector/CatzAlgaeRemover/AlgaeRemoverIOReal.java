// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover;

import static frc.robot.CatzSubsystems.CatzAlgaeEffector.CatzAlgaeRemover.AlgaeRemoverConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class AlgaeRemoverIOReal implements AlgaeRemoverIO {

  private final SparkMax algaeRemoverMotor;

  public AlgaeRemoverIOReal() {
    algaeRemoverMotor = new SparkMax(ALGAE_REMOVER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(STALL_CURRENT_LIMIT);
    globalConfig.idleMode(IdleMode.kBrake);
    globalConfig.voltageCompensation(12);

    algaeRemoverMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(AlgaeEffectorIOInputs inputs) {  }

  @Override
  public void runVolts(double speed) {
    System.out.println(speed + " ");
    algaeRemoverMotor.setVoltage(-speed);
  }

  @Override
  public void runVoltsBck(double speed) {
    algaeRemoverMotor.setVoltage(speed);
  }
}
