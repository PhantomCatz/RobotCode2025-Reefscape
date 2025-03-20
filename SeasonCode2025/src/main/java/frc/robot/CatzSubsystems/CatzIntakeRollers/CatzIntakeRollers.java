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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzSuperstructure.CoralState;
import lombok.Getter;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;

public class CatzIntakeRollers extends SubsystemBase {

    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();
    private int stuckCounter = 0;
    private RobotContainer container;

    public enum intakeRollersStates {
        A,
        B,
        C
    }

    @Setter @Getter
    private intakeRollersStates currentState = intakeRollersStates.C;
    private intakeRollersStates previousState = intakeRollersStates.C;

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
            currentState = intakeRollersStates.C;
            io.runIntakesIntakeMotor(0);
        }

        if(inputs.bbreakRampTriggered == true) {
            stuckCounter++;
            if(stuckCounter == 50) {
                stuckCounter = 0;
                io.runIntakesIntakeMotor(RAMP_UNSTUCK_SPD);
            }
        }

        if(currentState != previousState) {
            System.out.println("changed intake state!");
        }

        switch (currentState) {
            case A:
            break;
            case B:
            break;
            case C:
            break;
        }

        Logger.recordOutput("IntakeRoller/State", currentState);
        previousState = currentState;

    }
}
