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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeEffectorIOSparkmax implements AlgaeEffectorIO {

  private final DigitalInput beamBreak;

  private final SparkMax algaeEffectorMotor1;
  private final SparkMax algaeEffectorMotor2;

  public AlgaeEffectorIOSparkmax() {
    algaeEffectorMotor1 = new SparkMax(1, MotorType.kBrushless);
    algaeEffectorMotor2 = new SparkMax(2, MotorType.kBrushless);

    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig.smartCurrentLimit(50);
    globalConfig.idleMode(IdleMode.kBrake);
    globalConfig.voltageCompensation(12);

    algaeEffectorMotor1.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeEffectorMotor2.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    beamBreak = new DigitalInput(3);
  }

  @Override
  public void updateInputs(AlgaeEffectorIOInputs inputs) {
    inputs.bbreakTriggered = !beamBreak.get();
  }

  @Override
  public void runMotor(double speed, double speed2) {
    System.out.println(speed + " " + speed2);
    algaeEffectorMotor1.set(-speed);
    algaeEffectorMotor2.set(speed);
  }

  @Override
  public void runMotorBck(double speed) {
    algaeEffectorMotor1.set(speed);
    algaeEffectorMotor2.set(-speed);
  }

  @Override
  public void runMotorTop(double speed) {
    algaeEffectorMotor1.set(-speed);
  }

  @Override
  public void runMotorBottom(double speed) {
    algaeEffectorMotor2.set(speed);
  }
}
