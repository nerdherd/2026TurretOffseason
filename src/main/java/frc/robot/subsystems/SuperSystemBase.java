package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ParallelCommandGroup;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.nerd_logging.Reportable;

/**
 * Contains all helper functions of SuperSystem not specific to any subsystem
 * <p>
 * Helps to seperate the robot controlling code and the setup code
 * @see frc.robot.subsystems.SuperSystem
 */
public class SuperSystemBase implements Reportable {
    public static final ArrayList<TemplateSubsystem> subsystems = new ArrayList<>();
    public NerdDrivetrain swerveDrivetrain;

    public SuperSystemBase(NerdDrivetrain swerveDrivetrain) {
        this.swerveDrivetrain = swerveDrivetrain;
    }
    
    /**
     * Add a {@link frc.robot.subsystems.template.TemplateSubsystem TemplateSubsystem} to the internal array
     * @param subsystem
     */
    public static void registerSubsystem(TemplateSubsystem subsystem) {
        subsystems.add(subsystem);
    }
    
    /**
     * Apply an operation to every registered subsystem
     * @param f The operation to perform
     * @see {@link #registerSubsystem(TemplateSubsystem)}
     */
    public void applySubsystems(Consumer<TemplateSubsystem> f) {
        for (TemplateSubsystem subsystem : subsystems) f.accept(subsystem);
    }

    /**
     * Initializes the logging for every registered subsystem
     */
    @Override
    public void initializeLogging() {
        applySubsystems((s) -> s.initializeLogging());
    }

    // ------ helper functions ------ //
    /**
     * UNTESTED
     * <p>
     * Groups all inputted commands into a {@link org.wpilib.command2.ParallelCommandGroup ParallelCommandGroup} along with a {@link org.wpilib.command2.Commands#idle() Commands.idle()}
     * <p>
     * Basically, converts a group of commands (e.g. runonce) into a command that can run continuously
     * <p>
     * Allows things like {@link org.wpilib.command2.Command#andThen(Command...) .andThen(...)}
     * @param commands A collection of commands to group together
     * @return A "continuous" command
     */
    public ParallelCommandGroup continuousParallelCommand(Command... commands) {
        ParallelCommandGroup c2 = new ParallelCommandGroup(commands);
        c2.addCommands(Commands.idle());
        return c2;
    }

    /**
     * Runs {@link frc.robot.subsystems.template.TemplateSubsystem#applyMotorConfigs() TemplateSubsystem.applyMotorConfigs()} on every registered subsystem
     * @see {@link #registerSubsystem(TemplateSubsystem)}
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#applyMotorConfigs() TemplateSubsystem.applyMotorConfigs()}
     */
    public void reConfigureMotors() {
        applySubsystems((s) -> s.applyMotorConfigs());
    }

    /**
     * Sets the neutral mode for every registered subsystem
     * @param neutralMode The neutral mode to use
     * @see {@link #registerSubsystem(TemplateSubsystem)}
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#setNeutralMode(NeutralModeValue) TemplateSubsystem.setNeutralMode(NeutralModeValue)}
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
        applySubsystems((s) -> s.setNeutralMode(neutralMode));
    }

    /**
     * Fully stops all registered subsystems by putting them into neutral and disabling them
     * <p>
     * Subsystems do not reenable on their own
     * @see {@link #registerSubsystem(TemplateSubsystem)}
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#stop() TemplateSubsystem.stop()}
     */
    public void stop() {
        applySubsystems((s) -> s.stop());
    }   

    /**
     * Initializes all registered subsystems, if {@link frc.robot.subsystems.template.TemplateSubsystem#useSubsystem TemplateSubsystem.useSubsystem} is true for the subsystem
     * @see {@link #registerSubsystem(TemplateSubsystem)}
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#useSubsystem TemplateSubsystem.useSubsystem}
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#setEnabled(boolean) TemplateSubsystem.stop(boolean)}
     */
    public void initialize() {
        applySubsystems((s) -> s.setEnabled(s.useSubsystem));
    }

    /**
     * Sets the desired value for all registered subsystems to their defaults
     * @see {@link #registerSubsystem(TemplateSubsystem)}
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#getDefaultValue() TemplateSubsystem.getDefaultValue()}
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#setDesiredValue(double) TemplateSubsystem.setDesiredValue(double)}
     */
    public void resetSubsystemValues() {
        applySubsystems((s) -> s.setDesiredValue(s.getDefaultValue()));
    }
}
