package frc.robot.CatzSubsystems.Outtake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.CatzSubsystems.Outtake.OuttakeIOInputsAutoLogged;

public interface OuttakeIO {
    
    @AutoLog
    public static class OuttakeIOInputs {

        public boolean isIntakeIOMotorConnected = true;

        public double positionMechs = 0.0;
        public double velocityRpm = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

    }

    public default void runMotor(double Speed) {}
    
    public default void runMotorBck(double Speed) {}

    public default void runSparkMax(double Speed) {}

    public default void updateInputs(OuttakeIOInputsAutoLogged inputs) {}

    public default void setPID(double kP, double kI, double kD) {}

    public default void setFF(double kS, double kV, double kA) {}
    
    public default void runCharacterizationMotor(double input) {}
}

