package frc.robot.util.nerd_logging;

import static frc.robot.Constants.ROBOT_LOG_LEVEL;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import org.wpilib.util.struct.StructSerializable;
import org.jspecify.annotations.NonNull;
import org.jspecify.annotations.Nullable;
import org.wpilib.util.Alert;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;

import frc.robot.Constants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.nerd_logging.Reportable.LOG_LEVEL;

public class NerdLog {
	/** Contains the Runnables associated with each LOG_LEVEL. */
    private static HashMap<Reportable.LOG_LEVEL, ArrayList<Runnable>> logSuppliers = new HashMap<>();
	/** Contains the BaseStatusSignals associated with each network name. */
	private static HashMap<String, ArrayList<BaseStatusSignal>> refreshList = new HashMap<>();

	/** The time, in seconds, since the last update. */
	private static double timeLastPublished = 0.0;
	/** The number of loops processed. */
	private static int publishCount = 0;
	
	public static void periodic() {
		for (ArrayList<BaseStatusSignal> signals : refreshList.values())
			BaseStatusSignal.refreshAll(signals);
		
		// Updates logs on a certain interval.
		double currentTime = Timer.getMonotonicTimestamp();
		if (currentTime - timeLastPublished >= LoggingConstants.LOGGING_INTERVAL) {
			
			// Updates only the logs with the right LOG_LEVEL.
			LOG_LEVEL[] levels = LOG_LEVEL.values();
			for (int i = ROBOT_LOG_LEVEL.ordinal(); i < levels.length; i++) {

				// Updates the logs on certain loops, controlled by throttle.
				if (publishCount % levels[i].throttle == 0 && logSuppliers.containsKey(levels[i]))
					for (Runnable log : logSuppliers.get(levels[i]))
						log.run();
			}
			publishCount++;
			timeLastPublished = currentTime;
		}
	}

	/**
	 * Logs a double Supplier with units.
	 * @param name
	 * @param supplier
	 * @param unit
	 * @param loggingLevel
	 */
    public static void logNumber(@NonNull String name, Supplier<Double> supplier, String unit, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());

		Runnable logger = 
			() -> {DogLog.log(name, supplier.get(), unit);};
		logSuppliers.get(loggingLevel).add(logger);
	}
	
	/**
	 * Logs a BaseStatusSignal on a network.
	 * @param key
	 * @param name
	 * @param signal
	 * @param networkName
	 * @param loggingLevel
	 */
    public static void logSignal(@NonNull String name, BaseStatusSignal signal, String networkName, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = 
			() -> {DogLog.log(name, signal.getValueAsDouble(), signal.getUnits());};
		logSuppliers.get(loggingLevel).add(logger);
		if (!refreshList.containsKey(networkName)) refreshList.put(networkName, new ArrayList<>());
		refreshList.get(networkName).add(signal);
	}

	/**
	 * Logs a double Supplier.
	 * @param key
	 * @param name
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logNumber(@NonNull String name, Supplier<Double> supplier, LOG_LEVEL loggingLevel) {
		if (Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());

		Runnable logger = 
			() -> {DogLog.log(name, supplier.get());};
		logSuppliers.get(loggingLevel).add(logger);
	}

	/**
	 * Logs a double array Supplier.
	 * @param key
	 * @param name
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logNumberArray(@NonNull String name, Supplier<Double[]> supplier, LOG_LEVEL loggingLevel) {
		if (Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());

		Runnable logger = 
			() -> {
				Double[] Doubles = supplier.get();
				double[] doubles = new double[Doubles.length];
				for (int i = 0; i < Doubles.length; i++) doubles[i] = Doubles[i];
				DogLog.log(name, doubles);
			};
		logSuppliers.get(loggingLevel).add(logger);
	}

	/**
	 * Logs a double array Supplier with units.
	 * @param key
	 * @param name
	 * @param supplier
	 * @param unit
	 * @param loggingLevel
	 */
	public static void logNumberArray(@NonNull String name, Supplier<Double[]> supplier, String unit, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());

		Runnable logger = 
			() -> {
				Double[] Doubles = supplier.get();
				double[] doubles = new double[Doubles.length];
				for (int i = 0; i < Doubles.length; i++) doubles[i] = Doubles[i];
				DogLog.log(name, doubles);
			};
		logSuppliers.get(loggingLevel).add(logger);
	}

	/**
	 * Logs a boolean Supplier.
	 * @param key
	 * @param name
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logBoolean(@NonNull String name, Supplier<Boolean> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());

		Runnable logger =
			() -> {DogLog.log(name, supplier.get());};
		logSuppliers.get(loggingLevel).add(logger);
	}

	/**
	 * Logs a boolean array Supplier.
	 * @param key
	 * @param name
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logBooleanArray(@NonNull String name, Supplier<Boolean[]> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());

		Runnable logger = 
			() -> {
				Boolean[] objectArray = supplier.get();
				boolean[] array = new boolean[objectArray.length];
				for (int i = 0; i < objectArray.length; i++) array[i] = objectArray[i].booleanValue();
				DogLog.log(name, array);
			};
		logSuppliers.get(loggingLevel).add(logger);
	}

	/**
	 * Logs a String Supplier.
	 * @param key
	 * @param name
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logString(@NonNull String name, Supplier<String> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());

		Runnable logger = 
			() -> {DogLog.log(name, supplier.get());};
		logSuppliers.get(loggingLevel).add(logger);
	}
	
	/**
	 * Log a String array Supplier.
	 * @param key
	 * @param name
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logStringArray(@NonNull String name, Supplier<@Nullable String[]> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());

		Runnable logger = 
			() -> {DogLog.log(name, Objects.requireNonNull(supplier.get()));};
		logSuppliers.get(loggingLevel).add(logger);
	}

	/**
	 * Log a Sendable.
	 * @param key
	 * @param path
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logData(String path, TelemetryLoggable supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		Telemetry.log(path, supplier);
	}

	/**
	 * Logs a supplier for a StructSerializable implementation.
	 * @param key
	 * @param path
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logStructSerializable(@NonNull String path, Supplier<StructSerializable> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		if (!logSuppliers.containsKey(loggingLevel)) logSuppliers.put(loggingLevel, new ArrayList<>());
		Runnable logger = 
			() -> {DogLog.log(path, supplier.get());};
		logSuppliers.get(loggingLevel).add(logger);
	}

	/**
	 * Logs the state of the SwerveDrivetrain.
	 * @param key
	 * @param path
	 * @param supplier
	 * @param loggingLevel
	 */
	public static void logSwerveModules(String path, Supplier<SwerveDriveState> supplier, LOG_LEVEL loggingLevel) {
		if(Constants.ROBOT_LOG_LEVEL.ordinal() > loggingLevel.ordinal()) return;
		Telemetry.log(path, generateModuleSendable(supplier));
	}

	private static TelemetryLoggable generateModuleSendable(Supplier<SwerveDriveState> state) {
		return new TelemetryLoggable() {
			@Override
			public void logTo(TelemetryTable table) {
				table.log("Front Left Angle", state.get().ModuleVelocities[0].angle.getRadians());
				table.log("Front Left Velocity", state.get().ModuleVelocities[0].velocity);

				table.log("Front Right Angle", state.get().ModuleVelocities[1].angle.getRadians());
				table.log("Front Right Velocity", state.get().ModuleVelocities[1].velocity);

				table.log("Back Left Angle", state.get().ModuleVelocities[2].angle.getRadians());
				table.log("Back Left Velocity", state.get().ModuleVelocities[2].velocity);

				table.log("Back Right Angle", state.get().ModuleVelocities[3].angle.getRadians());
				table.log("Back Right Velocity", state.get().ModuleVelocities[3].velocity);

				table.log("Robot Angle", state.get().Pose.getRotation().getRadians());
			}

			@Override
			public String getTelemetryType() {
				return "SwerveDrive";
			}
		};
	}

	/**
	 * Reports an info statement through DogLog and DriverStation.
	 * @param message
	 */
	public static void reportInfo(String message) {
		DogLog.logFault(message, Alert.Level.LOW);
		DriverStationErrors.reportWarning(message, false);
	}
	
	/**
	 * Reports a warning through DogLog and DriverStation.
	 * @param message
	 */
	public static void reportWarning(String message) {
		DogLog.logFault(message, Alert.Level.MEDIUM);
		DriverStationErrors.reportWarning(message, true);
	}
	
	/**
	 * Reports an error message through DogLog and DriverStation.
	 * @param message
	 */
	public static void reportError(String message) {
		DogLog.logFault(message, Alert.Level.HIGH);
		DriverStationErrors.reportWarning(message, true);
	}

	/**
	 * Reports the number of values and StatusSignals logged.
	 */
	public static void reportLogCount() {
		String output = "";
		for (LOG_LEVEL level : LOG_LEVEL.values()) {
			if (logSuppliers.containsKey(level)) {
				output = output + level.name() + ": " + logSuppliers.get(level).size() + "\n";
			}
		}
		for (String network : refreshList.keySet()) {
			output = output + "SIGNALS (" + ((network.equals("")) ? "rio" : network) + "): " + refreshList.get(network).size() + "\n";
		}
		reportInfo(output);
	}
}
