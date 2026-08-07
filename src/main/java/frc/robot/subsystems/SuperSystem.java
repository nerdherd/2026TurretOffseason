package frc.robot.subsystems;



import java.util.ArrayList;
import java.util.function.Consumer;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.NerdyMath;
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
        
    }
}
