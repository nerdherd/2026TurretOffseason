package frc.robot.commands.autos;

import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import choreo.trajectory.EventMarker;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.SuperSystem.IntakeRollerMode;
import frc.robot.subsystems.SuperSystem.IntakeSlideMode;

public final class Autos {
    public static Selectable<Command> autoChooser = new Selectable<>();

    public static void initAutoChooser() {
        autoChooser.addDefault("Do Nothing", Commands.none());
        //EXAMPLE
        //autoChooser.add("Auto Name", AutoBuilder.buildAuto("PathPlanner Auto Name"));

        //TOP


        //MID
        

        //BOT
        Tunables.publish("Autos/Chooser", autoChooser);
    }
    

    public static void initBindings(SuperSystem superSystem) {
        //intake pivot
        new EventTrigger("Intake Sequence").onTrue(Commands.sequence(superSystem.setIntakeSlideCommand(IntakeSlideMode.OUT), Commands.waitSeconds(0.5), superSystem.setIntakeSlideCommand(IntakeSlideMode.HOLD)));

        //intake
        new EventTrigger("Intake Start").onTrue(superSystem.setIntakeRollerCommand(IntakeRollerMode.INTAKE));
        new EventTrigger("Intake Stop").onTrue(superSystem.setIntakeRollerCommand(IntakeRollerMode.STOP));
        
        //shooter
        new EventTrigger("Shoot Start").onTrue(superSystem.startShootAutoCommand());
        new EventTrigger("Shoot Stop").onTrue(superSystem.stopShootAutoCommand());
        NamedCommands.registerCommand("Shoot Start", superSystem.startShootAutoCommand());
        NamedCommands.registerCommand("Shoot Stop", superSystem.stopShootAutoCommand());



        



        // etc...
    }
    
}
