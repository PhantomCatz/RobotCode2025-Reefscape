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
import frc.robot.CatzSubsystems.CatzOuttake.CatzOuttake;
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

    public CatzIntakeRollers(RobotContainer container) {
        this.container = container;
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
                if(container.getCatzElevator().getElevatorPositionInch() < 4.0 && !DriverStation.isAutonomous()){
                    case_rampRollersIn();
                }
                if(DriverStation.isAutonomous()){
                    case_rampRollersIn();
                }
            break;
            case ANTIJAM:
                // adj_rampIntake(ANTIJAM_ROLLER_ROTATIONS);
                case_rampRollersAntiJam();
            break;
            case OUTTAKE:
                case_rampRollersOut();
            break;
            case STOP:
                case_rampRollersStop();
            break;
        }

        Logger.recordOutput("IntakeRoller/State", currentState.toString());
        previousState = currentState;

    }

    public double getIntakeRampRollerPosition() {
        return inputs.positionMechs;
    }

    private void case_rampRollersIn() {
        io.runIntakeRampMotor(RAMP_INTAKE_SPEED);

        if(inputs.bbreakRampFrntTriggered && inputs.bbreakRampBackTriggered) {
            stuckCounter++;
            if(stuckCounter >= 50) {
                stuckCounter = 0;
                currentState = intakeRollersStates.ANTIJAM;
            }
        }
        if(CatzOuttake.currentState == CatzOuttake.outtakeStates.STOP) {
            System.out.println("detected!!!11-1--120-3-oieisdjisddhcjsujgjhkishghjikhghjklhgvhjkgvhkjhgsjsghdughsjdhd");
            currentState = intakeRollersStates.STOP;
        }
    }

    private void case_rampRollersOut() {
        io.runIntakeRampMotor(RAMP_OUTAKE_SPEED);

    }

    // public void adj_rampIntake(double target) {
    //     io.adjustIntakeRamp(target);
    // }

    public void case_rampRollersAntiJam() {
        io.runIntakeRampMotor(RAMP_ADJUST_SPEED);

        if(!(inputs.bbreakRampFrntTriggered && inputs.bbreakRampBackTriggered)) {
            jamCounter++;
            if(jamCounter >= 20) {
                jamCounter = 0;
                currentState = intakeRollersStates.INTAKE;
            }
        }
    }

    private void case_rampRollersStop() {
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
