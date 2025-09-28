package frc.robot.CatzSubsystems.Bases;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

public class MotorIOReal implements MotorIO {

    // initialize follower if needed?
    private TalonFX leaderTalon;
    private TalonFX followerTalon;

    private DigitalInput Limit_Switch1;
    private DigitalInput Limit_Switch2;
    private DigitalInput Beam_Break_1;
    private DigitalInput Beam_Break_2;

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
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param mmP motion magic parameters
     */
    public MotorIOReal(TalonFX leader, double FL, Gains s0g, Gains s1g, MotionMagicParameters mmP) {

        leaderTalon = leader;
        followerTalon = null;

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
    public MotorIOReal(TalonFX leader, TalonFX followerMotor, double FL, Gains s0g, Gains s1g, MotionMagicParameters mmP) { // TODO we should look into finding ways to split it up.
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

    /**
     * basic, not done
     * 1 motor and 1 sensor
     * @param leader motor
     * @param di1 sensor
     * @param type the type of digital input, either "LS" for limit switch or "BB" for beam break
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param mmP motion magic parameters
     */
    public MotorIOReal(TalonFX leader, DigitalInput di1, String type, double FL, Gains s0g, Gains s1g, MotionMagicParameters mmP) {
        leaderTalon = leader;
        followerTalon = null;

        switch(type){
            case "LS":
                Limit_Switch1 = di1;
                break;
            case "BB":
                Beam_Break_1 = di1;
                break;
        }

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
     * 2 motors and 1 sensor
     * @param leader 1st motor
     * @param followerMotor 2nd motor, automatically set as same direction as leader
     * @param di1 sensor
     * @param type the type of digital input, either "LS" for limit switch or "BB" for beam break
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param mmP motion magic parameters
     */
    public MotorIOReal(TalonFX leader, TalonFX followerMotor, DigitalInput di1, String type, double FL, Gains s0g, Gains s1g, MotionMagicParameters mmP) {
        leaderTalon = leader;
        followerTalon = followerMotor;

        switch(type){
            case "LS":
                Limit_Switch1 = di1;
                break;
            case "BB":
                Beam_Break_1 = di1;
                break;
        }

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
    
    /**
     * basic, not done
     * 1 motor and 2 sensors
     * @param leader motor
     * @param di1 sensor 1
     * @param type1 the type of digital input for sensor 1, either "LS" for limit switch or "BB" for beam break and 1 or 2 after for the first or second
     * @param di2 sensor 2
     * @param type2 the type of digital input for sensor 2, either "LS" for limit switch or "BB" for beam break and 1 or 2 after for the first or second
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param mmP motion magic parameters
     */
    public MotorIOReal(TalonFX leader, DigitalInput di1, String type1, DigitalInput di2, String type2, double FL, Gains s0g, Gains s1g, MotionMagicParameters mmP) {
        leaderTalon = leader;
        followerTalon = null;

        switch(type1){
            case "LS1":
                Limit_Switch1 = di1;
                break;
            case "BB1":
                Beam_Break_1 = di1;
                break;
            case "LS2":
                Limit_Switch2 = di1;
                break;
            case "BB2":
                Beam_Break_2 = di1;
                break;
        }

        switch(type2){
            case "LS1":
                Limit_Switch1 = di2;
                break;
            case "BB1":
                Beam_Break_1 = di2;
                break;
            case "LS2":
                Limit_Switch2 = di2;
                break;
            case "BB2":
                Beam_Break_2 = di2;
                break;
        }

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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO should be a parameter


        leaderTalon.setPosition(0);

        leaderTalon.getConfigurator().apply(config, 1.0);

    } 
    
    /**
     * basic, not done
     * 2 motors and 2 sensors
     * @param leader 1st motor
     * @param followerMotor 2nd motor, automatically set as same direction as leader
     * @param di1 sensor
     * @param type1 the type of digital input, either "LS" for limit switch or "BB" for beam break and 1 or 2 after for the first or second
     * @param di2 sensor 2
     * @param type2 the type of digital input for sensor 2, either "LS" for limit switch or "BB" for beam break and 1 or 2 after for the first or second
     * @param FL Final Ration
     * @param s0g slot 0 gains
     * @param s1g slot 1 gains
     * @param mmP motion magic parameters
     */
    public MotorIOReal(TalonFX leader, TalonFX followerMotor, DigitalInput di1, String type1, DigitalInput di2, String type2, double FL, Gains s0g, Gains s1g, MotionMagicParameters mmP) { //TODO too many parameters, need to find a way to make this less cluttered
        leaderTalon = leader;
        followerTalon = followerMotor;

        switch(type1){ // TODO not fully a fan with doing a string to determine type, should use an overloaded constructor that takes in a limit switch or beam break object
            case "LS1":
                Limit_Switch1 = di1;
                break;
            case "BB1":
                Beam_Break_1 = di1;
                break;
            case "LS2":
                Limit_Switch2 = di1;
                break;
            case "BB2":
                Beam_Break_2 = di1;
                break;
        }

        switch(type2){
            case "LS1":
                Limit_Switch1 = di2;
                break;
            case "BB1":
                Beam_Break_1 = di2;
                break;
            case "LS2":
                Limit_Switch2 = di2;
                break;
            case "BB2":
                Beam_Break_2 = di2;
                break;
        }

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
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static; // How are we going to reconcile if it's a pivot vs a full elevator?

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

        inputs.positionInch = internalPositionRotations.getValueAsDouble() * Final_Ratio; //TODO Constants should be ALL_CAPS
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
        if (Limit_Switch1 != null) {
            inputs.isBotLimitSwitched1 = Limit_Switch1.get();
        }
        if (Limit_Switch2 != null) {
            inputs.isBotLimitSwitched2 = Limit_Switch1.get();
        }
        if (Beam_Break_1 != null) {
            inputs.bbreak1Triggered = Beam_Break_1.get();
        }
        if (Beam_Break_2 != null) {
            inputs.bbreak2Triggered = Beam_Break_2.get();
        }

    }

    @Override
    public void runSetpointFrwd(double setpointInches) {
        double setpointRotations = setpointInches / Final_Ratio;
        leaderTalon.setControl(positionControlFrwd.withPosition(setpointRotations));
    }

    @Override
    public void runSetpointBack(double setpointInches) {
        double setpointRotations = setpointInches / Final_Ratio;
        leaderTalon.setControl(positionControlBack.withPosition(setpointRotations));
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

    @Override
    public void runMotor(double speed) {
        System.out.println(speed);
        leaderTalon.setControl(new VoltageOut(speed));
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        leaderTalon.getConfigurator().apply(config);
    }

    public MotorIOInputs getMotorIOInputs() {
        return new MotorIO.MotorIOInputs();
    }

    public static class ControlRequestGetter { // TODO pretty cool! 
		public ControlRequest getVoltageRequest(Voltage voltage) {
			return new VoltageOut(voltage.in(Units.Volts)).withEnableFOC(false);
		}

		public ControlRequest getDutyCycleRequest(Dimensionless percent) {
			return new DutyCycleOut(percent.in(Units.Percent));
		}

		public ControlRequest getMotionMagicRequest(Angle mechanismPosition) {
			return new MotionMagicExpoVoltage(mechanismPosition).withSlot(0).withEnableFOC(true);
		}

		public ControlRequest getVelocityRequest(AngularVelocity mechanismVelocity) {
			return new VelocityTorqueCurrentFOC(mechanismVelocity).withSlot(1);
		}

		public ControlRequest getPositionRequest(Angle mechanismPosition) {
			return new PositionTorqueCurrentFOC(mechanismPosition).withSlot(2);
		}
	}

    public final void applySetpoint(Setpoint setpointToApply) {
		setpoint = setpointToApply;
		if (enabled) {
			setpointToApply.apply(this);
		}
	}


    private void setControl(ControlRequest request) {
		leaderTalon.setControl(request);
	}

    @Override
	public void setNeutralSetpoint() {
		setControl(new NeutralOut());
	}

	@Override
	public void setCoastSetpoint() {
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
