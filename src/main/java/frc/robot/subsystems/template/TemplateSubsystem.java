// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.template;

import static frc.robot.Constants.LoggingConstants.kSubsystemTab;

import java.util.ArrayList;
import java.util.function.BiConsumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;

import frc.robot.subsystems.SuperSystem;
import frc.robot.util.nerd_logging.NerdLog;
import frc.robot.util.nerd_logging.Reportable;

public class TemplateSubsystem extends SubsystemBase implements Reportable {
	/** 
	 * primary leading motor which the entire subsystem is based on
	 */
	public final TalonFX primaryMotor;

	/** 
	 * secondary motors that follow the primary<p></p>
	 * use {@link #addMotor(int, MotorAlignmentValue)}<p></p>
	 * <strong>if multiple motors need independent and responsive motion, make multiple subsystem objects</strong><p></p>
	 */
	public final ArrayList<TalonFX> secondaryMotors = new ArrayList<>();
	public final ArrayList<Follower> followRequests = new ArrayList<>();

	/** holds the configuration for both {@link #primaryMotor} and {@link #motor2} */
	protected TalonFXConfiguration configuration;

	/** position controller for {@link SubsystemMode#POSITION} */
	private MotionMagicVoltage positionController = null;
	/** velocity controller for {@link SubsystemMode#VELOCITY} */
	private VelocityVoltage velocityController = null;
	/** voltage controller for {@link SubsystemMode#VOLTAGE} */
	private VoltageOut voltageController = null;
	/** velocity controller for {@link SubsystemMode#PROFILED_VELOCITY} */
	private MotionMagicVelocityVoltage profiledVelocityController = null;

	/** brake/coast */
	private final NeutralOut neutralRequest = new NeutralOut();
	
	/** 
	 * target position or velocity depending on {@link SubsystemMode}
	 */
	private double desiredValue; 

	public final boolean useSubsystem;

	/** whether the subsystem will run {@link #periodic()} or use {@link #neutralRequest} */
	protected boolean enabled = false;

	/** used to indicate when the subsystem has an error, configured during debugging.  by default always false (reported at {@link LOG_LEVEL#ALL}) */
	public boolean _hasError = false;

	/** name of the subsystem */
	protected final String name;

	/** default value of the subsystem (e.g. position, velocity, voltage) */
	private final double defaultValue;

	public enum SubsystemMode {
		POSITION, 
		VELOCITY,
		VOLTAGE,
		PROFILED_VELOCITY
	}
	/** {@link SubsystemMode} of this subsystem */
	private final SubsystemMode mode;

	/**
	 * Used to define a subsystem with one or more motors, providing easy functions to control it
	 * 
	 * @param name - The name of the subsystem to be used for {@link #shuffleboardTab}
	 * @param motorID - The ID of the primary motor
	 * @param mode - The {@link SubsystemMode} for this subsystem, defining the control type
	 * @param defaultValue - Initial position or velocity depending on {@link SubsystemMode}
	 * @param useSubsystem - Whether or not to use this subsystem
	 */
	public TemplateSubsystem(String name, int motorID, SubsystemMode mode, double defaultValue, boolean useSubsystem) {
		this.primaryMotor = getMotor(motorID);
		this.defaultValue = defaultValue;
		this.desiredValue = defaultValue;
		this.useSubsystem = useSubsystem;
		this.mode = mode;
		this.name = name;

		switch (mode) {
			case POSITION:
				positionController = new MotionMagicVoltage(defaultValue);
				primaryMotor.setPosition(defaultValue);
				break;
			case VELOCITY: velocityController = new VelocityVoltage(defaultValue); break;
			case VOLTAGE: voltageController = new VoltageOut(defaultValue); break;
			case PROFILED_VELOCITY: profiledVelocityController = new MotionMagicVelocityVoltage(defaultValue); break;
			default: break;
		}
		
		SuperSystem.registerSubsystem(this);
	}
	
	/** 
	 * Add a secondary motor to this subsystem
	 * 
	 * @param motorID The ID of the motor to add
	 * @param reverse The {@link MotorAlignmentValue} of the motor, either aligned or opposed
	 * */
	public TemplateSubsystem addMotor(int motorID, MotorAlignmentValue reverse) {
		TalonFX motor = getMotor(motorID);
		secondaryMotors.add(motor);
		Follower followRequest = new Follower(primaryMotor.getDeviceID(), reverse);
		followRequests.add(followRequest);
		motor.setControl(followRequest);
		return this;
	}

	/** 
	 * Applies a configuration to every motor
	 * <p>
	 * Should be used on construction
	 * 
	 * @param configuration The {@link TalonFXConfiguration} to apply to the motors
	 * */
	public TemplateSubsystem configureMotors(TalonFXConfiguration configuration){
		this.configuration = configuration;
		DogLog.log(kSubsystemTab + name + "/motor configs", configuration.toString());
		applyMotorConfigs();
		return this;
	}

	/**
	 * Apply a consumer to all secondary motors
	 * @param f - A function that takes the motor and its ID
	 */
	public void applySecondaryMotors(BiConsumer<TalonFX, Integer> f) {
		for (int i = 0; i < secondaryMotors.size(); i++) f.accept(secondaryMotors.get(i), i);
	}

	@Override
	public void periodic() {
		if(!enabled || !useSubsystem) {
			stop();
			return;
		}

		follow();
		
		switch (mode) {
			case POSITION:
				primaryMotor.setControl(positionController.withPosition(this.desiredValue));
				if (configuration.MotionMagic.MotionMagicCruiseVelocity == 0.0) NerdLog.reportWarning(name + ": MM Cruise Velocity is 0.0");
				if (configuration.MotionMagic.MotionMagicAcceleration == 0.0) NerdLog.reportWarning(name + ": MM Acceleration is 0.0");
				break;
			case VELOCITY:
				if (Math.abs(this.desiredValue) <= 0.1) brake();
				else primaryMotor.setControl(velocityController.withVelocity(this.desiredValue));
				break;
			case VOLTAGE:
				if (Math.abs(this.desiredValue) > 12) NerdLog.reportWarning(name + ": voltage > 12");
				primaryMotor.setControl(voltageController.withOutput(this.desiredValue));
				break;
			case PROFILED_VELOCITY:
				if (Math.abs(this.desiredValue) <= 0.1) brake();
				else primaryMotor.setControl(profiledVelocityController.withVelocity(this.desiredValue).withAcceleration(configuration.MotionMagic.MotionMagicAcceleration));
				if (configuration.MotionMagic.MotionMagicAcceleration == 0.0) NerdLog.reportWarning(name + ": MM Acceleration is 0.0");
			default:
				break;
		}
	}

	// ------------------------------------ Helper Functions ------------------------------------ //
	
	/**
	 * Tries to get a motor by trying to find it on the rio. 
	 * If it isn't found, it tries again with the CANivore CANbus
	 * <p>
	 * Relies on completely unique ids for the entire robot network
	 * 
	 * @param id - ID of the motor to find
	 * @return The found motor, or a motor without a CANBus if not found
	 */
	private static TalonFX getMotor(int id) {
		TalonFX motor;
		if (id == 45) return new TalonFX(id, new CANBus("can_s3"));
		for (int i=0; i<5; i++) {
			motor = new TalonFX(id, new CANBus("can_s" + i));
			NerdLog.reportInfo("Trying CANBus " + i);
			if(motor.isConnected()) {
				return motor;
			}
			motor.close();
		} 
		
		NerdLog.reportError("Motor (ID " + id + ") could not be found");
		return new TalonFX(id, new CANBus());
	}

	/**
	 * Tries to get a CANdi by trying to find it on the rio.
	 * 
	 * @param id - ID of the CANdi to find
	 * @return The found CANdi, or a CANdi without a CANBus if not found
	 */
	public static CANdi getCANdi(int id) {
		CANdi candi;
		for (int i=0; i<5; i++) {
			candi = new CANdi(id, new CANBus("can_s" +i));
			NerdLog.reportInfo("Trying CANdi CANBus" + i);
			if(candi.isConnected()){
				return candi;
			}
			candi.close();
		}
		return new CANdi(id, new CANBus());
	}

	/**
	 * Tries to get a CANcoder by trying to find it on the rio.
	 * 
	 * @param id - ID of the CANcoder to find
	 * @return The found CANcoder, or a CANcoder without a CANBus if not found
	 */
	public static CANcoder getCANcoder(int id) {
		CANcoder cancoder;
		for (int i=0; i<5; i++) {
			cancoder = new CANcoder(id, new CANBus("can_s" +i));
			NerdLog.reportInfo("Trying CANconder CANBus" + i);
			if(cancoder.isConnected()){
				return cancoder;
			}
			cancoder.close();
		}
		return new CANcoder(id, new CANBus());
	}

	/**  
	 * Returns the subsystem mode in string form
	 * <p>
	 * Used for logging
	 * @return The subsystem mode in string form
	 * */
	public String getFlavorText() {
		switch (mode) {
			case POSITION: return "Position";
			case PROFILED_VELOCITY:
			case VELOCITY: return "Velocity";
			case VOLTAGE: return "Voltage";
			default: break;
		}
		return "";
	}

	/**  
	 * Returns the unit used based on the subsystem mode in string form
	 * <p>
	 * Used for logging
	 * @return The unit in string form
	 * */
	public String getUnit() {
		switch (mode) {
			case POSITION: return "rot";
			case PROFILED_VELOCITY:
			case VELOCITY: return "rps";
			case VOLTAGE: return "V";
			default: break;
		}
		return null;
	}

	/** 
	 * Applies motor configurations to all motors based on {@link #configuration} 
	 * */
	public void applyMotorConfigs(){
		primaryMotor.getConfigurator().apply(this.configuration);
		applySecondaryMotors((motor, id) -> motor.getConfigurator().apply(this.configuration));
	}
	
	/** 
	 * Sets and applies a {@link NeutralModeValue} to all motors
	 * */
	public void setNeutralMode(NeutralModeValue mode){
		this.configuration.MotorOutput.NeutralMode = mode;
		applyMotorConfigs();
	}
	
	/** 
	 * Brake or coast depending on current {@link NeutralModeValue} 
	 * */
	public void stop(){
		this.enabled = false;
		brake();
	}
	
	/** 
	 * Sets all motors to neutral out 
	 * */
	public void brake() {
		primaryMotor.setControl(neutralRequest);
		applySecondaryMotors((motor, i) -> motor.setControl(neutralRequest));
	}

	/** 
	 * Set the control of all secondary motors to follower
	 * */
	public void follow() {
		applySecondaryMotors((motor, i) -> motor.setControl(followRequests.get(i)));
	}

	/** 
	 * Enables or disables the subsystem 
	 * @see {@link #enabled}
	 * @param enabled Whether or not the subsystem should be enabled
	 */
	public void setEnabled(boolean enabled){
		this.enabled = enabled;
		if (!this.enabled) stop();
	}

	/**
	 * Disables the subsystem
	 * @see {@link #setEnabled(boolean)}
	 */
	public void disable() {
		this.setEnabled(false);
	}
	
	/**
	 * Enables the subsystem
	 * @see {@link #setEnabled(boolean)}
	 */
	public void enable(){
		this.setEnabled(true);
	}

	/**
	 * The main function to control the subsystem
	 * <p>
	 * Sets {@link #desiredValue}
	 * <p>
	 * Depending on {@link #mode}, this could be position, velocity, voltage, etc
	 * 
	 * @param value Value to set
	 */
	public void setDesiredValue(double value) {
		this.desiredValue = value;
	}

	/**
	 * Gets the desired value
	 * 
	 * @return {@link #desiredValue}
	 */
	public double getDesiredValue(){
		return desiredValue;
	}


	/**
	 * Gets the current value of the subsystem
	 * <p>
	 * The unit depends on the {@link #mode}; it could be position, velocity, or voltage
	 * 
	 * @return The current value based on the primary motor
	 */
	public BaseStatusSignal getCurrentValue() {
		switch (mode) {
			case POSITION: return primaryMotor.getPosition(false);
			case PROFILED_VELOCITY:
			case VELOCITY: return primaryMotor.getVelocity(false);
			case VOLTAGE: return primaryMotor.getMotorVoltage(false);
			default: break;
		}

		return null;
	}

	/**
	 * @return The current velocity of {@link #primaryMotor} 
	 */
	public double getCurrentVelocity() {
		return primaryMotor.getVelocity().getValueAsDouble();
	}

	/**
	 * @return The current position of {@link #primaryMotor} 
	 */
	public double getCurrentPosition() { 
		return primaryMotor.getPosition().getValueAsDouble();
	}

	/**
	 * True if {@link #getCurrentValue()} is less than {@code tolerance} away from {@link #desiredValue}
	 * @param tolerance the tolerance
	 * @return If the current value is within tolerance
	 */
	public boolean isValueWithinTolerance(double tolerance) {
		return Math.abs(getCurrentValue().getValueAsDouble() - getDesiredValue()) < tolerance;
	}

	/**
	 * @return Temperature of {@link #primaryMotor}
	 */
	public double getCurrentTemp(){
		return primaryMotor.getDeviceTemp().getValueAsDouble();
	}

	/**
	 * @return The {@link #defaultValue} of the subsystem
	 */
	public double getDefaultValue() {
		return defaultValue;
	}

	/**
	 * @return {@link #enabled}
	 */
	public boolean isEnabled() {
		return enabled;
	}

	// ------------------------------------ Command Functions ------------------------------------ //

	/**
	 * A command to set {@link #enabled}
	 * @param newEnabled
	 * @return A command that runs {@link #setEnabled(boolean)} with the value in {@code newEnabled}
	 */
	public Command setEnabledCommand(boolean newEnabled) {
		return Commands.runOnce(() -> setEnabled(newEnabled));
	}

	/**
	 * A command to set {@link #desiredValue}
	 * @param newValue
	 * @return A command that runs {@link #setDesiredValue(double)} with the value in {@code newValue}
	 */
	public Command setDesiredValueCommand(double newValue) {
		return Commands.runOnce(() -> setDesiredValue(newValue), this);
	}

	/**
	 * @return {@link #stop()} in command form
	 */
	public Command stopCommand() {
		return Commands.runOnce(() -> stop());
	}

	// ------------------------------------ Logging Functions ------------------------------------ //

	/** 
	 * Intialize shuffleboard logging on {@link #shuffleboardTab}
	 * <p>
	 * Logs values under {@link frc.robot.Constants.LoggingConstants#kSubsystemTab kSubsystemTab} + {@link #name}
	 */
    public void initializeLogging(){
		if (!useSubsystem) return;
        ///////////
        /// ALL ///
        ///////////
		NerdLog.logData(kSubsystemTab + name + "/Commands", this, LOG_LEVEL.ALL); 
		NerdLog.logBoolean(kSubsystemTab + name + "/Has Error", () -> _hasError, LOG_LEVEL.ALL);
		NerdLog.logSignal(kSubsystemTab + name + "/Supply Current", primaryMotor.getSupplyCurrent(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.ALL);
		
		
		//////////////
		/// MEDIUM ///
        //////////////
		NerdLog.logSignal(kSubsystemTab + name + "/Torque Current/Primary Motor", primaryMotor.getTorqueCurrent(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.MEDIUM);
		NerdLog.logSignal(kSubsystemTab + name + "/Motor Voltage/Primary Motor", primaryMotor.getMotorVoltage(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.MEDIUM);
		applySecondaryMotors((motor, i) -> {
			NerdLog.logSignal(kSubsystemTab + name + "/Torque Current/Secondary Motor " + i, motor.getTorqueCurrent(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.MEDIUM);
			NerdLog.logSignal(kSubsystemTab + name + "/Motor Voltage/Secondary Motor " + i, motor.getMotorVoltage(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.MEDIUM);
		});
		
        NerdLog.logBoolean(kSubsystemTab + name + "/Enabled", () -> this.enabled, Reportable.LOG_LEVEL.MEDIUM);
        NerdLog.logNumber(kSubsystemTab + name + "/Desired " + getFlavorText(), () -> getDesiredValue(), getUnit(), LOG_LEVEL.MEDIUM);

        //////////////
		/// MINIMAL //
        //////////////
        NerdLog.logSignal(kSubsystemTab + name + "/" + getFlavorText(), getCurrentValue(), primaryMotor.getNetwork().getName(), LOG_LEVEL.MINIMAL);
        NerdLog.logSignal(kSubsystemTab + name + "/Temperature/Primary Motor", primaryMotor.getDeviceTemp(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.MEDIUM);
		applySecondaryMotors((motor, i) -> 
			NerdLog.logSignal(kSubsystemTab + name + "/Temperature/Secondary Motor " + i, motor.getDeviceTemp(false), motor.getNetwork().getName(), LOG_LEVEL.MEDIUM)
		);
		NerdLog.logBoolean(kSubsystemTab + name + "/Connected/Primary Motor", primaryMotor::isConnected, LOG_LEVEL.MINIMAL);
		applySecondaryMotors((motor, i) -> 
			NerdLog.logBoolean(kSubsystemTab + name + "/Connected/Secondary Motor " + i, motor::isConnected, LOG_LEVEL.MINIMAL)
		);
    }
}
