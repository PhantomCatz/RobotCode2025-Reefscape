// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzOuttake;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


public class OuttakeIOSparkmax implements OuttakeIO {

  // private final DigitalInput beamBreakBck;
  // private final DigitalInput beamBreakFrnt;

  private final SparkMax outtakeMotor1;
  private final SparkMax outtakeMotor2;

  // private SparkBaseConfig globalConfig;

  public OuttakeIOSparkmax() {
    outtakeMotor1 = new SparkMax(1, MotorType.kBrushless);
    outtakeMotor2 = new SparkMax(2, MotorType.kBrushless);

    // globalConfig = new SparkBaseConfig();

    // globalConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    // globalConfig.idleMode(IdleMode.kBrake);

    // outtakeMotor1.configure(
    //     globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // outtakeMotor2.configure(
    //     globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    outtakeMotor1.set(-speed);
    outtakeMotor2.set(speed);
  }

  @Override
  public void runMotorBck(double speed) {
    outtakeMotor1.set(speed);
    outtakeMotor2.set(-speed);
  }

  @Override
  public void runMotorLeft(double speed) {
    outtakeMotor1.set(-speed);
  }

  @Override
  public void runMotorRight(double speed) {
    outtakeMotor2.set(speed);
  }
}
