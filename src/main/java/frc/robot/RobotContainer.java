package frc.robot;

import dev.doglog.DogLog;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.system.RobotController;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;

import frc.robot.Constants.ControllerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;

import frc.robot.util.controller.Controller;
import frc.robot.util.controller.Controller.Type;

import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable.LOG_LEVEL;


public class RobotContainer {

    public NerdDrivetrain swerveDrive;

    public SuperSystem superSystem;

    public PowerDistribution pdp =
        new PowerDistribution(0,1, ModuleType.REV);


    private final Controller driverController =
        new Controller(
            ControllerConstants.kDriverControllerPort,
            Type.PS4
        );


    private final Controller operatorController =
        new Controller(
            ControllerConstants.kOperatorControllerPort,
            Type.PS4
        );


    private final Controller testController =
        new Controller(
            ControllerConstants.kTestControllerPort,
            Type.Xbox360
        );


    private static boolean isRedSide = false;


    public RobotContainer() {

        swerveDrive = TunerConstants.createDrivetrain();

        if (Constants.USE_SUBSYSTEMS) {
            superSystem = new SuperSystem(swerveDrive);   
        }

        configureBindings();

        initializeLogging();

        NerdLog.get().reportInfo("Initialization Complete");
    }



    private void configureBindings() {

        configureDriverBindings();

        configureOperatorBindings();

    }



    private void configureDriverBindings() {

        // Add driver controls here

        /*
        Example:

        driverController.buttonA()
            .whileTrue(new SomeCommand());
        */

    }



    private void configureOperatorBindings() {

        // Add operator controls here

    }



    public void initializeDefaultCommands() {

        /*
         * Add drivetrain default command here
         *
         * Example:
         *
         * swerveDrive.setDefaultCommand(
         *      new SwerveJoystickCommand(...)
         * );
         */

    }



    public Command getAutonomousCommand() {

        // Replace with PathPlanner auto chooser later

        return Commands.none();

    }



    public static void refreshAlliance() {

        var alliance = MatchState.getAlliance();

        if(alliance.isPresent()) {
            isRedSide =
                alliance.get() == Alliance.RED;
        }

    }



    public static boolean IsRedSide() {
        return isRedSide;
    }



    public void initializeLogging() {

        NerdLog.get().logData(
            "Robot/Command Scheduler",
            CommandScheduler.getInstance(),
            LOG_LEVEL.MEDIUM
        );


        NerdLog.get().logNumber(
            "Robot/Battery Voltage",
            RobotController::getBatteryVoltage,
            LOG_LEVEL.MEDIUM
        );


        swerveDrive.initializeLogging();


        if(Constants.USE_SUBSYSTEMS) {
            superSystem.initializeLogging();
        }

    }

}

