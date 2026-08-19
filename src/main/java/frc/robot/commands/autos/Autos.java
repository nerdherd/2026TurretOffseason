package frc.robot.commands.autos;

import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable.LOG_LEVEL;

import static frc.robot.Constants.LoggingConstants.kAutosTab;

import com.pathplanner.lib.auto.NamedCommands;

public final class Autos {
    public static SendableChooser<Command> autoChooser = new SendableChooser<>();

    public static void initAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        // EXAMPLE
        // autoChooser.addOption("Auto Name", AutoBuilder.buildAuto("PathPlanner Auto Name"));

        // TOP
        // autoChooser.addOption("Top-S1MidDepot", AutoBuilder.buildAuto("Top-S1MidDepot"));

        // MID
        

        // BOT
        NerdLog.logData(kAutosTab + "/Selected Auto", autoChooser, LOG_LEVEL.MINIMAL);
    }

    public static void initNamedCommands(SuperSystem superSystem, NerdDrivetrain swerveDrive) {
        // SWERVE2
        NamedCommands.registerCommand("Reset Pose", swerveDrive.resetPoseWithAprilTags(0.2));

        // etc...
    }
    
}
