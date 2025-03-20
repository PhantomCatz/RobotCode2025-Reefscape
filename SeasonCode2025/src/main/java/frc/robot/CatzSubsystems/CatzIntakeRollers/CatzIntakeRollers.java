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
package frc.robot.CatzSubsystems.CatzIntakeRollers;
import static frc.robot.CatzSubsystems.CatzIntakeRollers.IntakeRollersConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;

public class CatzIntakeRollers extends SubsystemBase {

    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();
    private int stuckCounter = 0;
    private int jamCounter = 0;
    private int beamBreakFaultCounter = 0;
    private RobotContainer container;

    public enum intakeRollersStates {
        INTAKE,
        ANTIJAM,
        OUTTAKE,
        STOP
    }

    @Setter @Getter
    private intakeRollersStates currentState = intakeRollersStates.STOP;
    private intakeRollersStates previousState = intakeRollersStates.STOP;

    public CatzIntakeRollers() {
        if(isIntakeRollersDisabled) {
            io = new IntakeRollersIONull();
            System.out.println("Outtake Unconfigured");
        } else {
            switch (CatzConstants.hardwareMode) {
                case REAL:
                io = new IntakeRollersIOReal();
                System.out.println("Outtake Configured for Real");
                break;
                case REPLAY:
                io = new IntakeRollersIOReal() {};
                System.out.println("Outtake Configured for Replayed simulation");
                break;
                default:
                io = new IntakeRollersIONull();
                System.out.println("Outtake Unconfigured");
                break;
            }
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("RealInputs/IntakeRollers", inputs);

        if(DriverStation.isDisabled()) {
            currentState = intakeRollersStates.STOP;
            io.runIntakeRampMotor(0);
        }

        if(currentState != previousState) {
            System.out.println("changed intake state!");
            jamCounter = 0;
            stuckCounter = 0;
        }

        switch (currentState) {
            case INTAKE:
                run_rampIntakeIn();
            break;
            case ANTIJAM:
                // adj_rampIntake(ANTIJAM_ROLLER_ROTATIONS);
                adjusting_rampIntake();
            break;
            case OUTTAKE:
                run_rampIntakeOut();
            break;
            case STOP:
                run_rampIntakeOff();
            break;
        }

        Logger.recordOutput("IntakeRoller/State", currentState);
        previousState = currentState;

    }

    public double getIntakeRampRollerPosition() {
        return inputs.positionMechs;
    }

    private void run_rampIntakeIn() {
        io.runIntakeRampMotor(RAMP_INTAKE_SPEED);

        if(inputs.bbreakRampFrntTriggered && inputs.bbreakRampBackTriggered) {
            stuckCounter++;
            if(stuckCounter >= 50) {
                stuckCounter = 0;
                currentState = intakeRollersStates.ANTIJAM;
            }
        }
    }

    private void run_rampIntakeOut() {
        io.runIntakeRampMotor(RAMP_OUTAKE_SPEED);

    }

    // public void adj_rampIntake(double target) {
    //     io.adjustIntakeRamp(target);
    // }

    public void adjusting_rampIntake() {
        io.runIntakeRampMotor(RAMP_ADJUST_SPEED);

        if(!(inputs.bbreakRampFrntTriggered && inputs.bbreakRampBackTriggered)) {
            jamCounter++;
            if(jamCounter >= 20) {
                jamCounter = 0;
                currentState = intakeRollersStates.INTAKE;
            }
        }
    }

    private void run_rampIntakeOff() {
        io.runIntakeRampMotor(0.0);
    }

    public Command intake() {
        return runOnce(() -> currentState = intakeRollersStates.INTAKE);
    }

    public Command antiJam() {
        return runOnce(() -> currentState = intakeRollersStates.ANTIJAM);
    }

    public Command outtake() {
        return runOnce(() -> currentState = intakeRollersStates.OUTTAKE);
    }

    public Command stopIntaking() {
        return runOnce(() -> currentState = intakeRollersStates.STOP);
    }
}
