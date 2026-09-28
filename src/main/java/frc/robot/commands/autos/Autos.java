package frc.robot.commands.autos;

import org.wpilib.tunable.Selectable;
import org.wpilib.command2.Command;

// import com.pathplanner.lib.auto.NamedCommands;

public final class Autos {
    public static Selectable<Command> autoChooser = new Selectable<>();

    // public static void initAutoChooser() {
    //     autoChooser.setDefaultOption("Do Nothing", Commands.none());
    //     // EXAMPLE
    //     // autoChooser.addOption("Auto Name", AutoBuilder.buildAuto("PathPlanner Auto Name"));

    //     // TOP
    //     // autoChooser.addOption("Top-S1MidDepot", AutoBuilder.buildAuto("Top-S1MidDepot"));

    //     // MID
        

    //     // BOT
    //     NerdLog.logData(kAutosTab + "/Selected Auto", autoChooser, LOG_LEVEL.MINIMAL);
    // }

    // public static void initNamedCommands(SuperSystem superSystem, NerdDrivetrain swerveDrive) {
    //     // SWERVE2
    //     NamedCommands.registerCommand("Reset Pose", swerveDrive.resetPoseWithAprilTags(0.2));

    //     // etc...
    // }
    
}
