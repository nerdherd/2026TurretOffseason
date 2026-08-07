// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.template;

import static frc.robot.Constants.LoggingConstants.kSubsystemTab;

import java.util.ArrayList;
import java.util.function.BiConsumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable;

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

	private boolean _logTorqueCurrent = false;

	/** name of the subsystem */
	protected final String name;

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
	 * 
	 * @param name - used for {@link #shuffleboardTab}
	 * @param motor1ID - used for {@link #primaryMotor}
	 * @param motor2ID - used for {@link #motor2}
	 * @param reverseMotor2 - whether to have {@link #motor2} oppose {@link #primaryMotor}
	 * @param mode - the {@link SubsystemMode} for this subsystem
	 * @param defaultValue - initial position or velocity depending on {@link SubsystemMode}
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
	
	/** add a secondary motor to this subsystem */
	public TemplateSubsystem addMotor(int motorID, MotorAlignmentValue reverse) {
		TalonFX motor = getMotor(motorID);
		secondaryMotors.add(motor);
		Follower followRequest = new Follower(primaryMotor.getDeviceID(), reverse);
		followRequests.add(followRequest);
		motor.setControl(followRequest);
		return this;
	}

	/** applies configuration to motors; should be used on construction */
	public TemplateSubsystem configureMotors(TalonFXConfiguration configuration){
		this.configuration = configuration;
		DogLog.log(kSubsystemTab + name + "/motor configs", configuration.toString());
		applyMotorConfigs();
		return this;
	}

	/**
	 * apply a consumer to all secondary motors
	 * @param f - a function that takes the motor and its ID
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
				if (configuration.MotionMagic.MotionMagicCruiseVelocity == 0.0) NerdLog.get().reportWarning(name + ": MM Cruise Velocity is 0.0");
				if (configuration.MotionMagic.MotionMagicAcceleration == 0.0) NerdLog.get().reportWarning(name + ": MM Acceleration is 0.0");
				break;
			case VELOCITY:
				if (Math.abs(this.desiredValue) <= 0.1) brake();
				else primaryMotor.setControl(velocityController.withVelocity(this.desiredValue));
				break;
			case VOLTAGE:
				if (Math.abs(this.desiredValue) > 12) NerdLog.get().reportWarning(name + ": voltage > 12");
				primaryMotor.setControl(voltageController.withOutput(this.desiredValue));
				break;
			case PROFILED_VELOCITY:
				if (Math.abs(this.desiredValue) <= 0.1) brake();
				else primaryMotor.setControl(profiledVelocityController.withVelocity(this.desiredValue).withAcceleration(configuration.MotionMagic.MotionMagicAcceleration));
				if (configuration.MotionMagic.MotionMagicAcceleration == 0.0) NerdLog.get().reportWarning(name + ": MM Acceleration is 0.0");
			default:
				break;
		}
	}

	public TemplateSubsystem logTorqueCurrent() {
		_logTorqueCurrent = true;
		return this;
	}

	// ------------------------------------ Helper Functions ------------------------------------ //
	
	/**
	 * gets a motor by trying to find it on the rio. if it isn't found, it tries again with the CANivore CANbus
	 * relies on completely unique ids for the entire robot network
	 * @param id - id of the motor to find
	 * @return the found motor
	 */
	private static TalonFX getMotor(int id) {
		TalonFX motor = new TalonFX(id);
		if (!motor.isConnected()) {
			motor.close();
			motor = new TalonFX(id, TunerConstants.kCANBus);
		}
		return motor;
	}

	/** used for logging, essentially returns the subsystem mode in string form */
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

	/** applies motor configurations based on {@link #configuration} */
	public void applyMotorConfigs(){
		primaryMotor.getConfigurator().apply(this.configuration);
		applySecondaryMotors((motor, id) -> motor.getConfigurator().apply(this.configuration));
	}
	
	/** sets and applies a {@link NeutralModeValue} to motors */
	public void setNeutralMode(NeutralModeValue mode){
		primaryMotor.setNeutralMode(mode);
		applySecondaryMotors((motor, id) -> motor.setNeutralMode(mode));
	}
	
	/** brake or coast depending on current {@link NeutralModeValue} */
	public void stop(){
		this.enabled = false;
		brake();
	}
	
	/** sets all motors to neutral out */
	public void brake() {
		primaryMotor.setControl(neutralRequest);
		applySecondaryMotors((motor, i) -> motor.setControl(neutralRequest));
	}

	/** set secondary motors to follower */
	public void follow() {
		applySecondaryMotors((motor, i) -> motor.setControl(followRequests.get(i)));
	}

	/** 
	 * enables or disables the subsystem 
	 * @see {@link #enabled} 
	 */
	public void setEnabled(boolean enabled){
		this.enabled = enabled;
		if (!this.enabled) stop();
	}

	/**
	 * disables the subsystem
	 */
	public void disable() {
		this.setEnabled(false);
	}
	
	/**
	 * enables the subsystem
	 */
	public void enable(){
		this.setEnabled(true);
	}

	/**
	 * sets {@link #desiredValue}
	 * @param value - value to set
	 */
	public void setDesiredValue(double value) {
		this.desiredValue = value;
	}

	/**
	 * @return {@link #desiredValue}
	 */
	public double getDesiredValue(){
		return desiredValue;
	}


	/**
	 * {@link #desiredValue}
	 * @return the position or velocity based on motor1
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
	 * could be useful for atPoint checks
	 * @return the current velocity of {@link #primaryMotor} 
	 */
	public double getCurrentVelocity() {
		return primaryMotor.getVelocity().getValueAsDouble();
	}

	public double getCurrentPosition() { 
		return primaryMotor.getPosition().getValueAsDouble();
	}

	/**
	 * @return temperature of {@link #primaryMotor}
	 */
	public double getCurrentTemp(){
		return primaryMotor.getDeviceTemp().getValueAsDouble();
	}

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
	 * @param newEnabled
	 * @return {@link #enable()} in command form
	 */
	public Command setEnabledCommand(boolean newEnabled) {
		return Commands.runOnce(() -> setEnabled(newEnabled));
	}

	/**
	 * @param newValue
	 * @return {@link #setDesiredValue(double)} in command form
	 */
	public Command setDesiredValueCommand(double newValue) {
		return Commands.runOnce(() -> setDesiredValue(newValue));
	}

	/**
	 * @return {@link #stop()} in command form
	 */
	public Command stopCommand() {
		return Commands.runOnce(() -> stop());
	}

	// ------------------------------------ Logging Functions ------------------------------------ //

	/** 
	 * intialize shuffleboard logging on {@link #shuffleboardTab}
	 * @see {@link Reportable#addNumber(ShuffleboardTab, String, java.util.function.DoubleSupplier, frc.robot.util.logging.Reportable.LOG_LEVEL)}
	 * @see {@link Reportable#addBoolean(ShuffleboardTab, String, java.util.function.BooleanSupplier, frc.robot.util.logging.Reportable.LOG_LEVEL)}
	 * @see {@link Reportable#addString(ShuffleboardTab, String, java.util.function.Supplier, frc.robot.util.logging.Reportable.LOG_LEVEL)}
	 */
    public void initializeLogging(){
		if (!useSubsystem) return;
        ///////////
        /// ALL ///
        ///////////
		NerdLog.get().logData(kSubsystemTab + name + "/Commands", this, LOG_LEVEL.ALL); 
		
        NerdLog.get().logNumber(kSubsystemTab + name + "/Desired " + getFlavorText(), () -> getDesiredValue(), getUnit(), LOG_LEVEL.ALL);
		NerdLog.get().logBoolean(kSubsystemTab + name + "/Has Error", () -> _hasError, LOG_LEVEL.ALL);

		NerdLog.get().logSignal(kSubsystemTab + name + "/Torque Current/Primary Motor", primaryMotor.getTorqueCurrent(false), primaryMotor.getNetwork().getName(), (_logTorqueCurrent) ? LOG_LEVEL.MINIMAL : LOG_LEVEL.ALL);
		applySecondaryMotors((motor, i) -> 
			NerdLog.get().logSignal(kSubsystemTab + name + "/Torque Current/Secondary Motor " + i, primaryMotor.getTorqueCurrent(false), primaryMotor.getNetwork().getName(), (_logTorqueCurrent) ? LOG_LEVEL.MINIMAL : LOG_LEVEL.ALL));

		NerdLog.get().logSignal(kSubsystemTab + name + "/Supply Current", primaryMotor.getSupplyCurrent(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.ALL);

		//////////////
		/// MEDIUM ///
        //////////////
        NerdLog.get().logBoolean(kSubsystemTab + name + "/Enabled", () -> this.enabled, Reportable.LOG_LEVEL.MEDIUM);
		NerdLog.get().logSignal(kSubsystemTab + name + "/Motor Voltage", primaryMotor.getMotorVoltage(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.MEDIUM);
        
        //////////////
		/// MINIMAL //
        //////////////
        NerdLog.getNT().logSignal(kSubsystemTab + name + "/" + getFlavorText(), getCurrentValue(), primaryMotor.getNetwork().getName(), LOG_LEVEL.MINIMAL);
        NerdLog.get().logSignal(kSubsystemTab + name + "/Temperature/Primary Motor", primaryMotor.getDeviceTemp(false), primaryMotor.getNetwork().getName(), LOG_LEVEL.MEDIUM);
		applySecondaryMotors((motor, i) -> 
			NerdLog.get().logSignal(kSubsystemTab + name + "/Temperature/Secondary Motor " + i, motor.getDeviceTemp(false), motor.getNetwork().getName(), LOG_LEVEL.MEDIUM)
		);
		NerdLog.getNT().logBoolean(kSubsystemTab + name + "/Connected/Primary Motor", primaryMotor::isConnected, LOG_LEVEL.MINIMAL);
		applySecondaryMotors((motor, i) -> 
			NerdLog.getNT().logBoolean(kSubsystemTab + name + "/Connected/Secondary Motor " + i, motor::isConnected, LOG_LEVEL.MINIMAL)
		);
    }
}
