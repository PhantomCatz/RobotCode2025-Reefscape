package frc.robot.CatzSubsystems.CatzOuttake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class OuttakeIOSparkmax implements OuttakeIO { 

    private final DigitalInput beamBreakBck;
    private final DigitalInput beamBreakFrnt;

    private final SparkMax outtakeMotor1;
    private final SparkMax outtakeMotor2; 

    public OuttakeIOSparkmax() {
        outtakeMotor1 = new SparkMax(1, MotorType.kBrushless);
        outtakeMotor2 = new SparkMax(2, MotorType.kBrushless);

        SparkMaxConfig globalConfig = new SparkMaxConfig();

        globalConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
        globalConfig.idleMode(IdleMode.kBrake);

        outtakeMotor1.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outtakeMotor2.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        beamBreakBck = new DigitalInput(3);
        beamBreakFrnt = new DigitalInput(6);

    }


    @Override
    public void updateInputs(OuttakeIOInputs inputs) {
        inputs.bbreakFrntTriggered = !beamBreakFrnt.get();
        inputs.bbreakBackTriggered = !beamBreakBck.get();
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
    public void runMotorLeft(double speed){
        outtakeMotor1.set(-speed);
    }

    @Override
    public void runMotorRight(double speed){
        outtakeMotor2.set(speed);
    }





}
