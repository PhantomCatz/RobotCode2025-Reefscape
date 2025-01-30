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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class OuttakeIOSparkmax implements OuttakeIO {

  // private final DigitalInput beamBreakBck;
  // private final DigitalInput beamBreakFrnt;

  private final SparkMax OuttakeLeftMtr;
  private final SparkMax OuttakeRightMtr;

  // private SparkBaseConfig globalConfig;

  public OuttakeIOSparkmax() {
    OuttakeLeftMtr = new SparkMax(1, MotorType.kBrushless);
    OuttakeRightMtr = new SparkMax(2, MotorType.kBrushless);

    SparkMaxConfig globalConfig = new SparkMaxConfig();


    globalConfig.smartCurrentLimit(50);
    globalConfig.idleMode(IdleMode.kBrake);
    globalConfig.voltageCompensation(12);

    OuttakeLeftMtr.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    OuttakeRightMtr.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // beamBreakBck = new DigitalInput(3);
    // beamBreakFrnt = new DigitalInput(6);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    // inputs.bbreakFrntTriggered = !beamBreakFrnt.get();
    // inputs.bbreakBackTriggered = !beamBreakBck.get();
  }

  @Override
  public void runMotor(double speed, double speed2) {

    OuttakeLeftMtr.set(-speed);
    OuttakeRightMtr.set(speed);
  }

  @Override
  public void runMotorBck(double speed) {
    OuttakeLeftMtr.set(speed);
    OuttakeRightMtr.set(-speed);
  }

  @Override
  public void runMotorLeft(double speed) {
    OuttakeLeftMtr.set(-speed);
  }

  @Override
  public void runMotorRight(double speed) {
    OuttakeRightMtr.set(speed);
  }
}
