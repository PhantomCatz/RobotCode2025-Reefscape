// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzAlgaeEffector;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class AlgaeEffectorIOSparkmax implements AlgaeEffectorIO {

  private final DigitalInput beamBreak;

  private final SparkMax algaeEffectorMotor1;
  private final SparkMax algaeEffectorMotor2;

  public AlgaeEffectorIOSparkmax() {
    algaeEffectorMotor1 = new SparkMax(1, MotorType.kBrushless);
    algaeEffectorMotor2 = new SparkMax(2, MotorType.kBrushless);

    beamBreak = new DigitalInput(3);
  }

  @Override
  public void updateInputs(AlgaeEffectorIOInputs inputs) {
    inputs.bbreakTriggered = !beamBreak.get();
  }

  @Override
  public void runMotor(double speed, double speed2) {

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
