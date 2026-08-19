package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.LoggingConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable;

public class SuperSystem implements Reportable {
    public static final ArrayList<TemplateSubsystem> subsystems = new ArrayList<>();
    public NerdDrivetrain swerveDrivetrain;

    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    public static void registerSubsystem(TemplateSubsystem subsystem) {
        subsystems.add(subsystem);
    }
    
    public void applySubsystems(Consumer<TemplateSubsystem> f) {
        for (TemplateSubsystem subsystem : subsystems) f.accept(subsystem);
    }


    // ------------------------------------ subsystems ------------------------------------ //
    public void reConfigureMotors() {
        applySubsystems((s) -> s.applyMotorConfigs());
    }
    // /**
    //  * Drives to the scoring position and raises the arm at the same time.
    //  *
    //  * <p>Cancels itself if the driver takes over translation control.
    //  *
    //  * @return the composed command.
    //  */
    // public Command intake() {
    //     return Commands.parallel(
    //         intakeRoller.setDesiredValueCommand(11),
    //         intakeHoldTeleop()
    //         );
    // }
    //
    // public Command stopIntaking() {
    //     return Commands.parallel(
    //         intakeRoller.setDesiredValueCommand(0),
    //         stopIntakeHold()
    //     );
    // }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        applySubsystems((s) -> s.setNeutralMode(neutralMode));
    }
    /**
     * fully stops all subsystems by putting them into neutral and disabling them
     * subsystems do not reenable on their own
     * @return a command to stop
     */
    public Command stop() {
        return Commands.runOnce(() -> {
            applySubsystems((s) -> s.stop());
        });
    }   

    public void initialize() {
        applySubsystems((s) -> s.setEnabled(s.useSubsystem));
    }

    public void resetSubsystemValues() {
        applySubsystems((s) -> s.setDesiredValue(s.getDefaultValue()));
    }

    // ------------------------------------ logging ------------------------------------ //
    @Override
    public void initializeLogging() {
        applySubsystems((s) -> s.initializeLogging());
        NerdLog.logData(LoggingConstants.kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    }
}
