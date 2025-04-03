//------------------------------------------------------------------------------------
// 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project. 
//
//        "6 hours of debugging can save you 5 minutes of reading documentation."
//
//------------------------------------------------------------------------------------
package frc.robot.CatzSubsystems.CatzElevator;

/** Add your docs here. */
public class ElevatorIONull implements ElevatorIO {

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {}

    @Override
    public void runMotor(double Speed) {}

    @Override
    public void runMotorBck(double Speed) {}

    @Override
    public void runCurrent(double amps) {}

    @Override
    public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    @Override
    public void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    @Override
    public void setFF(double kS, double kV, double kA) {}

    @Override
    public void runCharacterizationMotor(double input) {}

    @Override
    public void runSetpointUp(double setpointRotations) {}

    @Override
    public void runSetpointDown(double setpointRotations) {}

    @Override
    public void setPosition(double pos) {}

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}

    @Override
    public void stop() {

    }
}
