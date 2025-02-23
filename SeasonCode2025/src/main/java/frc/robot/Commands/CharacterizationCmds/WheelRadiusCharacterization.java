// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Commands.CharacterizationCmds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.DriveConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

/**************************************************************************************************
 *
 *
 *
 * WheelRadiusCharacterization
 *
 *
 *
 **************************************************************************************************/

public class WheelRadiusCharacterization extends Command {
  private static final LoggedTunableNumber characterizationSpeed =
      new LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 1);
  private static final double driveRadius = DriveConstants.DRIVE_CONFIG.driveBaseRadius();
  private static final DoubleSupplier gyroYawRadsSupplier =
      () -> CatzRobotTracker.getInstance().getEstimatedPose().getRotation().getRadians();

  @RequiredArgsConstructor
  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);
    private final int value;
  }

  private final CatzDrivetrain drive;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;
  private double currentEffectiveWheelRadius = 0.0;
  private double[] startWheelPositions;

  // -----------------------------------------------------------------------------------------------
  //
  // Wheel Radius Characterization Constructor
  //
  // -----------------------------------------------------------------------------------------------
  public WheelRadiusCharacterization(CatzDrivetrain drive, Direction omegaDirection) {
    this.drive = drive;
    this.omegaDirection = omegaDirection;
    addRequirements(drive);
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Initialize
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void initialize() {
    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;
    omegaLimiter.reset(0);
  }

  // -----------------------------------------------------------------------------------------------
  //
  // Execute
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void execute() {
    // ---------------------------------------------------------------------------------------------
    // Run drive at velocity, calculate and log effective wheel radius from yaw and wheel positions
    // ---------------------------------------------------------------------------------------------
    drive.runWheelRadiusCharacterization(
        omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()));
    accumGyroYawRads +=
        Math.abs(MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads));
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();

    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]) / 4.0;
    }
    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / (averageWheelPosition * DriveConstants.MODULE_GAINS_AND_RATIOS.driveReduction());

    System.out.println("accum angle: " + accumGyroYawRads);
    System.out.println("avg wheel: " + averageWheelPosition);
    System.out.println("curr: " + currentEffectiveWheelRadius);

    Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    Logger.recordOutput(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  } // end of execute()

  // -----------------------------------------------------------------------------------------------
  //
  // End
  //
  // -----------------------------------------------------------------------------------------------
  @Override
  public void end(boolean interrupted) {
    drive.stopDriving();
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
