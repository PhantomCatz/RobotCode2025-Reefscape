package frc.robot.CatzSubsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
    
    @AutoLog
    public static class OuttakeIOInputs {

        public boolean isIntakeIOMotorConnected = true;

        public double PositionMechs = 0.0;
        public double VelocityRpm = 0.0;
        public double AppliedVolts = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;

    }

    public default void runMotor(double Speed) {}

    public default void runSparkMax(double Speed) {}

    public default void updateInputs(OuttakeIOInputs inputs) {}

    public default void setPID(double kP, double kI, double kD) {}

    public default void setFF(double kS, double kV, double kA) {}
    
    public default void runCharacterizationMotor(double input) {}
}

