package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ParallelCommandGroup;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.nerd_logging.Reportable;

public class SuperSystemBase implements Reportable {
    public static final ArrayList<TemplateSubsystem> subsystems = new ArrayList<>();
    public NerdDrivetrain swerveDrivetrain;

    public SuperSystemBase(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    public static void registerSubsystem(TemplateSubsystem subsystem) {
        subsystems.add(subsystem);
    }
    
    public void applySubsystems(Consumer<TemplateSubsystem> f) {
        for (TemplateSubsystem subsystem : subsystems) f.accept(subsystem);
    }

    @Override
    public void initializeLogging() {
        applySubsystems((s) -> s.initializeLogging());
    }

    // ------ helper functions ------ //
    public ParallelCommandGroup continuousParallelCommand(Command... commands) {
        ParallelCommandGroup c2 = new ParallelCommandGroup(commands);
        c2.addCommands(Commands.idle());
        return c2;
    }

    public void reConfigureMotors() {
        applySubsystems((s) -> s.applyMotorConfigs());
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        applySubsystems((s) -> s.setNeutralMode(neutralMode));
    }
    /**
     * fully stops all subsystems by putting them into neutral and disabling them
     * subsystems do not reenable on their own
     * @return a command to stop
     */
    public void stop() {
        applySubsystems((s) -> s.stop());
    }   

    public void initialize() {
        applySubsystems((s) -> s.setEnabled(s.useSubsystem));
    }

    public void resetSubsystemValues() {
        applySubsystems((s) -> s.setDesiredValue(s.getDefaultValue()));
    }
}
