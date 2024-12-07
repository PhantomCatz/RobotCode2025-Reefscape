// 2637
// https://github.com/PhantomCatz/

package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.*;

import org.littletonrobotics.junction.Logger;


public class ModuleIORealFoc implements ModuleIO {
  // Hardware
  private final TalonFX driveTalon;
  private final CANSparkMax steerSparkMax;
  private final DutyCycleEncoder steerAbsoluteMagEnc;
  private final DigitalInput magEncPWMInput;
  private final Rotation2d absoluteEncoderOffset;

  // Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveSupplyCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  // drive Controller Configs
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);
  private final PIDController steerFeedback = new PIDController(moduleGainsAndRatios.steerkP(), 0.0, moduleGainsAndRatios.steerkD());

  // Status Code Initialization
  private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

  ModuleConfig m_config;
  public ModuleIORealFoc(ModuleConfig config) {
    m_config = config;
    // Init drive controllers from config constants
    driveTalon = new TalonFX(config.driveID());

    // Restore Factory Defaults
    driveTalon.getConfigurator().apply(new TalonFXConfiguration());

    // Config Motors Current Limits assume FOC is included with motors
    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0; 
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Gain Setting
    driveTalonConfig.Slot0.kP = moduleGainsAndRatios.drivekP();
    driveTalonConfig.Slot0.kD = moduleGainsAndRatios.drivekD();
    driveTalonConfig.Slot0.kS = moduleGainsAndRatios.driveFFkS();
    driveTalonConfig.Slot0.kV = moduleGainsAndRatios.driveFFkV();

    //check if drive motor is initialized correctly
    for(int i=0;i<5;i++){
      initializationStatus = driveTalon.getConfigurator().apply(driveTalonConfig);
      if(!initializationStatus.isOK())
          System.out.println("Failed to Configure CAN ID" + config.driveID());
    }


    // Init Steer controllers and steer encoder from config constants
    magEncPWMInput = new DigitalInput(config.absoluteEncoderChannel());
    steerAbsoluteMagEnc = new DutyCycleEncoder(magEncPWMInput);
    absoluteEncoderOffset = Rotation2d.fromRotations(config.absoluteEncoderOffset());
    steerSparkMax = new CANSparkMax(config.steerID(), MotorType.kBrushless);
    steerSparkMax.enableVoltageCompensation(12.0);
    steerSparkMax.setSecondaryCurrentLimit(12);
    steerSparkMax.burnFlash();

    steerFeedback.enableContinuousInput(-Math.PI, Math.PI);

    // Assign 100hz Signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();



    // Set Update Frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent
    );

    // Optimize bus utilization
    driveTalon.optimizeBusUtilization(0, 1.0);
  } //end of ModuleIORealFoc constructor

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // Refresh Drive Kraken status signals
    inputs.isDriveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent)
            .isOK();

    //Drive Input variable refresh
    inputs.drivePositionUnits     = drivePosition.getValueAsDouble();
    inputs.driveVelocityRPS       = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts      = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    // Refresh steer Motor Values
    inputs.isSteerMotorConnected   = true;
    inputs.steerAbsoluteEncPosition = Rotation2d.fromRotations(steerAbsoluteMagEnc.getAbsolutePosition());
    inputs.steerAbsolutePosition   = Rotation2d.fromRotations(steerAbsoluteMagEnc.getAbsolutePosition() - absoluteEncoderOffset.getRotations());
    inputs.steerPosition           = Rotation2d.fromRotations(steerSparkMax.getEncoder().getPosition());
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerSparkMax.getEncoder().getVelocity());
    inputs.steerBusVoltage         = steerSparkMax.getBusVoltage();


    inputs.odometryDrivePositionsMeters = new double[] {drivePosition.getValueAsDouble() * driveConfig.wheelRadius()};
    inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerAbsolutePosition};
  } //end of updateInputs(ModuleIOInputs inputs)

  public void runDriveVolts(double volts) {
    driveTalon.setControl(voltageControl.withOutput(volts));
  }

  public void runSteerVolts(double volts) {
    steerSparkMax.setVoltage(volts);
  }

  @Override
  public void runCharacterization(double input) {
    driveTalon.setControl(currentControl.withOutput(input));
  }

  @Override
  public void runDriveVelocityRPSIO(double velocityMetersPerSec) {
    driveTalon.setControl(velocityTorqueCurrentFOC.withVelocity(velocityMetersPerSec));
  }

  public void runSteerPercentOutput(double percentOutput) {
    steerSparkMax.set(percentOutput);
  }

  @Override
  public void runSteerPositionSetpoint(double currentAngleRads, double targetAngleRads) {
      //calculate steer pwr
      //negative steer power because of coordinate system 
    double percentOutput = -steerFeedback.calculate(currentAngleRads, targetAngleRads); // Right in terms of the coordinate system was not right in terms of the motor
    runSteerPercentOutput(percentOutput);

     Logger.recordOutput("Module " + m_config.driveID() + "/steer volts", percentOutput);
     Logger.recordOutput("Module " + m_config.driveID() + "/steer current Angle", currentAngleRads);
     Logger.recordOutput("Module " + m_config.driveID() + "/steer Target Angle", targetAngleRads);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
  }

  @Override
  public void setSteerPID(double kP, double kI, double kD) {
    steerFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveNeutralModeIO(NeutralModeValue type) {
    driveTalonConfig.MotorOutput.NeutralMode = type;
    driveTalon.getConfigurator().apply(driveTalonConfig);
  }

  @Override
  public void setSteerNeutralModeIO(IdleMode type) {
      steerSparkMax.setIdleMode(type);
  }
}
