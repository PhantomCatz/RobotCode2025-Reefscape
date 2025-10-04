package frc.robot.CatzSubsystems.Bases;

import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.CatzSubsystems.Bases.MotorIO.MotorIOInputs;
import frc.robot.CatzSubsystems.Bases.MotorIOReal.ControlRequestGetter;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

public class ServoMotorIOReal implements ServoMotorIO{
    
    private TalonFX leaderTalon;
    private TalonFX followerTalon;

    private Gains slot0_gainsM;
    private Gains slot1_gainsM;

    private MotionMagicParameters motionMagicParameters;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final MotionMagicVoltage positionControlFrwd = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
    private final MotionMagicVoltage positionControlBack = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

    private final StatusSignal<Angle> internalPositionRotations;
    private final StatusSignal<AngularVelocity> velocityRps;
    private final List<StatusSignal<Voltage>> appliedVoltage;
    private final List<StatusSignal<Current>> supplyCurrent;
    private final List<StatusSignal<Current>> torqueCurrent;
    private final List<StatusSignal<Temperature>> tempCelsius;

    private double Final_Ratio;

    private final ControlRequestGetter requestGetter = new ControlRequestGetter();

    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5, java.util.concurrent.TimeUnit.MILLISECONDS, queue);
    
    private Setpoint setpoint = Setpoint.withNeutralSetpoint();
    private boolean enabled = true;
    
    
    /**
     * basic, not done
     * 1 motor
     * @param leader motor
     * @param FL Final Ratio
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param mmP motion magic parameters
     */
    public ServoMotorIOReal(TalonFX motor, double FL, Gains s0g, Gains s1g, MotionMagicParameters mmP) {

        leaderTalon = motor;

        Final_Ratio = FL;

        slot0_gainsM = s0g;
        slot1_gainsM = s1g;

        motionMagicParameters = mmP;

        internalPositionRotations = leaderTalon.getPosition();
        velocityRps = leaderTalon.getVelocity();
        appliedVoltage = List.of(leaderTalon.getMotorVoltage(), followerTalon.getMotorVoltage());
        supplyCurrent = List.of(leaderTalon.getSupplyCurrent(), followerTalon.getSupplyCurrent());
        torqueCurrent = List.of(leaderTalon.getTorqueCurrent(), followerTalon.getTorqueCurrent());
        tempCelsius = List.of(leaderTalon.getDeviceTemp(), followerTalon.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        tempCelsius.get(0),
        tempCelsius.get(1));

        // PID configs
        config.Slot0.kS = slot0_gainsM.kS();
        config.Slot0.kV = slot0_gainsM.kV();
        config.Slot0.kA = slot0_gainsM.kA();
        config.Slot0.kP = slot0_gainsM.kP();
        config.Slot0.kI = slot0_gainsM.kI();
        config.Slot0.kD = slot0_gainsM.kD();
        config.Slot0.kG = slot0_gainsM.kG();
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.Slot1.kS = slot1_gainsM.kS();
        config.Slot1.kV = slot1_gainsM.kV();
        config.Slot1.kA = slot1_gainsM.kA();
        config.Slot1.kP = slot1_gainsM.kP();
        config.Slot1.kI = slot1_gainsM.kI();
        config.Slot1.kD = slot1_gainsM.kD();
        config.Slot1.kG = slot1_gainsM.kG();
        config.Slot1.GravityType = GravityTypeValue.Elevator_Static;


        // Supply Current Limits
        config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Motion Magic Parameters
        config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
        config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
        config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


        leaderTalon.setPosition(0);

        leaderTalon.getConfigurator().apply(config, 1.0);

    }

    /**
     * basic, not done
     * 2 motors
     * @param leader 1st motor
     * @param followerMotor 2nd motor, automatically set as same direction as leader
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param mmP motion magic parameters
     */
    public ServoMotorIOReal(TalonFX leader, TalonFX followerMotor, double FL, Gains s0g, Gains s1g, MotionMagicParameters mmP) {
        leaderTalon = leader;
        followerTalon = followerMotor;

        Final_Ratio = FL;

        slot0_gainsM = s0g;
        slot1_gainsM = s1g;

        motionMagicParameters = mmP;

        internalPositionRotations = leaderTalon.getPosition();
        velocityRps = leaderTalon.getVelocity();
        appliedVoltage = List.of(leaderTalon.getMotorVoltage(), followerTalon.getMotorVoltage());
        supplyCurrent = List.of(leaderTalon.getSupplyCurrent(), followerTalon.getSupplyCurrent());
        torqueCurrent = List.of(leaderTalon.getTorqueCurrent(), followerTalon.getTorqueCurrent());
        tempCelsius = List.of(leaderTalon.getDeviceTemp(), followerTalon.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        tempCelsius.get(0),
        tempCelsius.get(1));

        // PID configs
        config.Slot0.kS = slot0_gainsM.kS();
        config.Slot0.kV = slot0_gainsM.kV();
        config.Slot0.kA = slot0_gainsM.kA();
        config.Slot0.kP = slot0_gainsM.kP();
        config.Slot0.kI = slot0_gainsM.kI();
        config.Slot0.kD = slot0_gainsM.kD();
        config.Slot0.kG = slot0_gainsM.kG();
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.Slot1.kS = slot1_gainsM.kS();
        config.Slot1.kV = slot1_gainsM.kV();
        config.Slot1.kA = slot1_gainsM.kA();
        config.Slot1.kP = slot1_gainsM.kP();
        config.Slot1.kI = slot1_gainsM.kI();
        config.Slot1.kD = slot1_gainsM.kD();
        config.Slot1.kG = slot1_gainsM.kG();
        config.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        // Supply Current Limits, does this need a varialbe input into it?
        config.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Motion Magic Parameters
        config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
        config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
        config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


        leaderTalon.setPosition(0);
        followerTalon.setPosition(0);

        leaderTalon.getConfigurator().apply(config, 1.0);
        followerTalon.getConfigurator().apply(config, 1.0);

        followerTalon.setControl(new Follower(leaderTalon.getDeviceID(), false));

        positionControlFrwd.EnableFOC = true;
        positionControlFrwd.Slot = 0;

        positionControlBack.EnableFOC = true;
        positionControlBack.Slot = 1;
    }

    public void updateInputs(MotorIOInputs inputs) {
        inputs.isLeaderMotorConnected =
            BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0))
            .isOK();

        inputs.isFollowerMotorConnected = // TODO Some mechanisms may not have a followerer for their subtsystem rendering this redundant
            BaseStatusSignal.refreshAll(
                appliedVoltage.get(1),
                supplyCurrent.get(1),
                torqueCurrent.get(1),
                tempCelsius.get(1))
            .isOK();

        inputs.positionInch = internalPositionRotations.getValueAsDouble() * Final_Ratio; //TODO Constants should be ALL_CAPS // Yuyhun said that because we get it from constructor that it should be lowercase
        inputs.velocityInchPerSec = velocityRps.getValueAsDouble() * Final_Ratio;
        inputs.appliedVolts =     appliedVoltage.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
        inputs.supplyCurrentAmps = supplyCurrent.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
        inputs.torqueCurrentAmps = torqueCurrent.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
        inputs.tempCelcius =         tempCelsius.stream()
                                                .mapToDouble(StatusSignal::getValueAsDouble)
                                                .toArray();
    }

    @Override
    public void stop() {
        leaderTalon.setControl(new DutyCycleOut(0.0));
    }

    @Override
    public void setPosition(double pos) {
        leaderTalon.setPosition(pos);
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        leaderTalon.getConfigurator().apply(config);
    }

    @Override
    public void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        config.Slot1.kP = kP;
        config.Slot1.kI = kI;
        config.Slot1.kD = kD;
        config.Slot1.kS = kS;
        config.Slot1.kV = kV;
        config.Slot1.kA = kA;
        config.Slot1.kG = kG;
        leaderTalon.getConfigurator().apply(config);
    }

    @Override
    public void setMotionMagicParameters(double vel, double accel, double jerk) {
        config.MotionMagic.MotionMagicCruiseVelocity = vel;
        config.MotionMagic.MotionMagicAcceleration = accel;
        config.MotionMagic.MotionMagicJerk = jerk;
        leaderTalon.getConfigurator().apply(config);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (followerTalon == null) {
            leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }
        else {
            leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }
    }

    public MotorIOInputs getMotorIOInputs() {
        return new MotorIO.MotorIOInputs();
    }
 
    // public static class ControlRequestGetter {
	// 	public ControlRequest getVoltageRequest(Voltage voltage) {
	// 		return new VoltageOut(voltage.in(Units.Volts)).withEnableFOC(false);
	// 	}

	// 	public ControlRequest getDutyCycleRequest(Dimensionless percent) {
	// 		return new DutyCycleOut(percent.in(Units.Percent));;
	// 	}

	// 	public ControlRequest getMotionMagicRequest(Angle mechanismPosition) {
	// 		return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0).withEnableFOC(true);
	// 	}

	// 	public ControlRequest getVelocityRequest(AngularVelocity mechanismVelocity) {
	// 		return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
	// 	}

	// 	public ControlRequest getPositionRequest(Angle mechanismPosition) {
	// 		return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
	// 	}
	// }

    public final void applySetpoint(Setpoint setpointToApply) {
		setpoint = setpointToApply;
		if (enabled) {
			setpointToApply.apply(this);
		}
	}
    
    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        config.MotorOutput.NeutralMode = mode;
    }


    private void setControl(ControlRequest request) {
		leaderTalon.setControl(request);
	}

    @Override
	public void setNeutralOut() {
		setControl(new NeutralOut());
	}

	@Override
	public void setCoastOut() {
		setControl(new CoastOut());
	}

	@Override
	public void setVoltageSetpoint(Voltage voltage) {
		setControl(requestGetter.getVoltageRequest(voltage));
	}

	@Override
	public void setDutyCycleSetpoint(Dimensionless percent) {
		setControl(requestGetter.getDutyCycleRequest(percent));
	}

	@Override
	public void setMotionMagicSetpoint(Angle mechanismPosition) {
		setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
	}

	@Override
	public void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
		setControl(requestGetter.getVelocityRequest(mechanismVelocity));
	}

	@Override
	public void setPositionSetpoint(Angle mechanismPosition) {
		setControl(requestGetter.getPositionRequest(mechanismPosition));
	}

	@Override
	public void setCurrentPosition(Angle mechanismPosition) {
		threadPoolExecutor.submit(() -> {
			leaderTalon.setPosition(mechanismPosition);
		});
	}
}
