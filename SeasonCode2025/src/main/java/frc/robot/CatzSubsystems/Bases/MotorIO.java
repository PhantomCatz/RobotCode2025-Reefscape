package frc.robot.CatzSubsystems.Bases;

import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;

public interface MotorIO {

  @AutoLog
  public static class MotorIOInputs {

    public boolean isLeaderMotorConnected = false;
    public boolean isFollowerMotorConnected = false;
    public boolean isBotLimitSwitched1 = false;
    public boolean isBotLimitSwitched2 = false;
    public boolean bbreak1Triggered = false;
    public boolean bbreak2Triggered = false;

    public double positionInch = 0.0;
    public double absoluteEncoderPositionRads = 0.0;
    public double relativeEncoderPositionRads = 0.0;
    public double velocityInchPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

  }

  public default void updateInputs(MotorIOInputs inputs) {}

  public default void runMotor(double Speed) {}

  public default void runMotorBck(double Speed) {}

  public default void runCurrent(double amps) {}

  public default void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public default void setFF(double kS, double kV, double kA) {}

  public default void runCharacterizationMotor(double input) {}

  public default void runSetpointFrwd(double setpointInches) {}

  public default void runSetpointBack(double setpointInches) {}

  public default void setPosition(double pos) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}

  public default void stop() {}

  public default MotorIOInputs getMotorIOInputs() {return new MotorIO.MotorIOInputs();}

  public default void setMotionMagicSetpoint(Angle mechanismPosition) {}

  public default void setVelocitySetpoint(AngularVelocity mechanismVelocity) {}

  public default void setDutyCycleSetpoint(Dimensionless percent) {}

  public default void setPositionSetpoint(Angle mechanismPosition) {}

  public default void setVoltageSetpoint(Voltage voltage) {}

  public default void setCoastSetpoint() {}

  public default void setNeutralSetpoint() {}

  public default void setCurrentPosition(Angle mechanismPosition) {}

  public default void applySetpoint(Setpoint setpointToApply) {}

  public enum Mode {
		IDLE,
		VOLTAGE,
		MOTIONMAGIC,
		VELOCITY,
		DUTY_CYCLE,
		POSITIONPID;

		/**
		 * Gets whether the control mode is based on position. Motion Magic and Position PID control count as position.
		 *
		 * @return True if in position control, false if not.
		 */
		public boolean isPositionControl() {
			return switch (this) {
				case MOTIONMAGIC, POSITIONPID -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is based on velocity.
		 *
		 * @return True if in velocity control, false if not.
		 */
		public boolean isVelocityControl() {
			return switch (this) {
				case VELOCITY -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is neutral. Only Idle counts as neutral
		 *
		 * @return True if in velocity control, false if not.
		 */
		public boolean isNeutralControl() {
			return switch (this) {
				case IDLE -> true;
				default -> false;
			};
		}

		/**
		 * Gets whether the control mode is based on voltage. Voltage and Duty Cycle control count as voltage.
		 *
		 * @return True if in voltage control, false if not.
		 */
		public boolean isVoltageControl() {
			return switch (this) {
				case VOLTAGE, DUTY_CYCLE -> true;
				default -> false;
			};
		}
	}

  public static class Setpoint {
		private final UnaryOperator<MotorIO> applier;
		public final Mode mode;
		public final double baseUnits;

		/**
		 * Creates a setpoint with a given applier, control mode, and base units equivalent.
		 *
		 * @param applier What to apply to MotorIO when the setpoint is set.
		 * @param mode Control mode to register for this setpoint.
		 * @param baseUnits Setpoint's target in it's base form of units as a double.
		 */
		private Setpoint(UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
			this.applier = applier;
			this.mode = mode;
			this.baseUnits = baseUnits;
		}

		/**
		 * Creates a setpoint with a completely custom applier, control mode, and base units.
		 *
		 * @param applier What to apply to MotorIO when the setpoint is set.
		 * @param mode Control mode to register for this setpoint.
		 * @param baseUnits Setpoint's target in it's base form of units as a double.
		 */
		public static Setpoint withCustomSetpoint(UnaryOperator<MotorIO> applier, Mode mode, double baseUnits) {
			return new Setpoint(applier, mode, baseUnits);
		}

		/**
		 * Creates a setpoint to use motion magic control to go to a position.
		 *
		 * @param motionMagicSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withMotionMagicSetpoint(Angle motionMagicSetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setMotionMagicSetpoint(motionMagicSetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.MOTIONMAGIC, motionMagicSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to use PID control to go to a position.
		 *
		 * @param positionSetpoint Posiiton to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withPositionSetpoint(Angle positionSetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setPositionSetpoint(positionSetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.POSITIONPID, positionSetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to go to a velocity.
		 *
		 * @param velocitySetpoint Velocity to go to in mechanism units.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVelocitySetpoint(AngularVelocity velocitySetpoint) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setVelocitySetpoint(velocitySetpoint);
				return io;
			};
			return new Setpoint(applier, Mode.VELOCITY, velocitySetpoint.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to run at a voltage.
		 *
		 * @param voltage Voltage to run at.
		 * @return A new Setpoint.
		 */
		public static Setpoint withVoltageSetpoint(Voltage voltage) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setVoltageSetpoint(voltage);
				return io;
			};
			return new Setpoint(applier, Mode.VOLTAGE, voltage.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to run at a percent of maximum voltage.
		 *
		 * @param percent Percent to run at.
		 * @return A new Setpoint.
		 */
		public static Setpoint withDutyCycleSetpoint(Dimensionless percent) {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setDutyCycleSetpoint(percent);
				return io;
			};
			return new Setpoint(applier, Mode.DUTY_CYCLE, percent.baseUnitMagnitude());
		}

		/**
		 * Creates a setpoint to idle.
		 *
		 * @return A new Setpoint.
		 */
		public static Setpoint withNeutralSetpoint() {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setNeutralSetpoint();
				return io;
			};
			return new Setpoint(applier, Mode.IDLE, 0.0);
		}

		/**
		 * Creates a setpoint to coast.
		 *
		 * @return A new Setpoint.
		 */
		public static Setpoint withCoastSetpoint() {
			UnaryOperator<MotorIO> applier = (MotorIO io) -> {
				io.setCoastSetpoint();
				return io;
			};
			return new Setpoint(applier, Mode.IDLE, 0.0);
		}

		public void apply(MotorIO io) {
			applier.apply(io);
		}
	}
}
