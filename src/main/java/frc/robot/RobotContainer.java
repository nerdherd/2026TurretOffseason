package frc.robot;

import dev.doglog.DogLog;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

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
        new PowerDistribution(1, ModuleType.kRev);


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

        var alliance = DriverStation.getAlliance();

        if(alliance.isPresent()) {
            isRedSide =
                alliance.get() == DriverStation.Alliance.Red;
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

