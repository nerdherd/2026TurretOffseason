package frc.robot;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.system.RobotController;

import dev.doglog.DogLog;

import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.networktables.StringSubscriber;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.Autos;
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
            Autos.initNamedCommands(superSystem, swerveDrive);
        }

        Subsystems.init(); // required to initialize the class or else java lazy loading just doesn't
        Autos.initAutoChooser();
        initializeLogging();

        NerdLog.reportInfo("Initialization Complete");
    }

    public void initDefaultCommands_teleop() {
        SwerveJoystickCommand swerveJoystickCommand =
            new SwerveJoystickCommand(
                swerveDrive,
                // Horizontal Translation
                () -> -driverController.getLeftY(), 
                // Vertical Translation
                () -> -driverController.getLeftX(), 
                // Turn
                () -> -driverController.getRightX(), 
                // use turn to angle
                () -> driverController.getBumperRight(),
                // turn to angle target direction, 0.0 to use manual
                () -> 0.0,
                // robot oriented adjustment (dpad)
                () -> new Translation2d(),
                // joystick drive field oriented
                () -> true, 
                // tow supplier
                () -> driverController.getBumperLeft(), 
                // precision/programmer mode :)
                () -> driverController.getTriggerLeftAxis()
            );
        swerveDrive.setDefaultCommand(swerveJoystickCommand);
    }

    public void initDefaultCommands_test() {
        swerveDrive.removeDefaultCommand();
    }

    public void configureBindings_teleop() {
        configureDriverBindings_teleop();
        configureOperatorBindings_teleop();
    }

    ///////////////////////
    // Driver bindings
    //////////////////////
    private void configureDriverBindings_teleop() {
        // Add driver controls here

        /*
        Example:

        driverController.buttonA()
            .whileTrue(new SomeCommand());
        */

        if (Constants.USE_SUBSYSTEMS) { /* bindings for subsystems */}
    }
    
    ///////////////////////
    // Operator bindings
    //////////////////////
    private void configureOperatorBindings_teleop() {
        
        // Add operator controls here
        if (Constants.USE_SUBSYSTEMS) { /* bindings for subsystems */}
    }

    public void configureBindings_test() {
        // TODO move these into Controller later
        testController.buttonRight()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Right Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Right Test", "bye")));
        testController.buttonDown()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Down Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Down Test", "bye")));
        testController.buttonUp()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Up Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Up Test", "bye")));
        testController.buttonLeft()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Left Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Left Test", "bye")));

        testController.bumperLeft()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Bumper L Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Bumper L Test", "bye")));
        testController.bumperRight()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Bumper R Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Bumper R Test", "bye")));
        
        testController.triggerLeft()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Trigger L Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Trigger L Test", "bye")));
        testController.triggerRight()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Trigger R Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Trigger R Test", "bye")));

    testController.dpadUp()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Up Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Up Test", "bye")));
        testController.dpadRight()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Right Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Right Test", "bye")));
        testController.dpadDown()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Down Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Down Test", "bye")));
        testController.dpadLeft()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Dpad Left Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Dpad Left Test", "bye")));

        testController.controllerLeft()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Controller Left Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Controller Left Test", "bye")));
        testController.controllerRight()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Controller Right Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Controller Right Test", "bye")));
        
        testController.joystickLeft()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Left Joy Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Left Joy Test", "bye")));
        testController.joystickRight()
        .onTrue(Commands.runOnce(() -> SmartDashboard.putString("Button Right Joy Test", "hi")))
        .onFalse(Commands.runOnce(() -> SmartDashboard.putString("Button Right Joy Test", "bye")));
    }

    public Command getAutonomousCommand() {
        return Autos.autoChooser.getSelected();
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

    public StringSubscriber printLog = null;
    public void initializeLogging() {
        if (printLog == null) printLog = DogLog.tunable("Print", "", (value) -> NerdLog.reportInfo("" + value));
        NerdLog.logData(
            "Robot/PDP", 
            pdp, 
            LOG_LEVEL.ALL);
        NerdLog.logData(
            "Robot/Command Scheduler",
            CommandScheduler.getInstance(),
            LOG_LEVEL.MEDIUM
        );
        NerdLog.logNumber(
            "Robot/Battery Voltage",
            RobotController::getBatteryVoltage,
            LOG_LEVEL.MEDIUM
        );
        NerdLog.logNumber(
            "Match Info/Shift Time", 
            () -> {shiftTime = allianceShiftTime(); return shiftTime;}, 
            LOG_LEVEL.MINIMAL
        );

        swerveDrive.initializeLogging();
        if(Constants.USE_SUBSYSTEMS) {
            superSystem.initializeLogging();
        }
        NerdLog.reportLogCount();
    }

    private static boolean gameEnded = false;
    public static double shiftTime = 0.0;
  /**
   * Displays a countdown for alliance shifts. NOT 100% ACCURATE
   * @return the number of seconds in the current phase, and the phase name
   */
    public static double allianceShiftTime() {
        // if (!DriverStation.isFMSAttached()) { DogLog.forceNT.log("Match Info/Shift Name", "DriverStation not attached"); return 0.0; };
        boolean wonAuto = true;
        if (Constants.ROBOT_LOG_LEVEL == LOG_LEVEL.MEDIUM) {
            String data = MatchState.getGameData().get();
            if (!data.isEmpty()) switch (data.charAt(0)) {
                case 'B': wonAuto = !isRedSide; break;
                case 'R': wonAuto = isRedSide; break;
                default: break;
            } 
            DogLog.log("Match Info/Won Auto?", wonAuto);
        }

        double time = MatchState.getMatchTime();
        DogLog.log("Match Info/time", time);

        if (RobotState.isAutonomous()) {
            if (time < 0.0) { DogLog.log("Match Info/Shift Name", (gameEnded) ? "Good Job Team!" : "Get Ready..."); return 0.0; }
            DogLog.log("Match Info/Shift Name", "Auto");
            gameEnded = false;
            return time;
        } else if (RobotState.isTeleop()) {
            if (time < 0.0) { DogLog.log("Match Info/Shift Name", (gameEnded) ? "Good Job Team!" : "Good Luck! -nerdherd"); return 0.0; }
            else if (time >= 130.0) { DogLog.log("Match Info/Shift Name", "Transition"); return time - 130; } // transition
            else if (time >= 30.0) { 
                int shift = (int)((130 - time) / 25) + 1; 
                DogLog.log("Match Info/Shift Name", "Shift " + shift + " " + (((shift % 2 == 1) == wonAuto) ? "Feeding" : "Scoring")); return (time - 30) % 25; 
            } // shifts 1-4
            else { DogLog.log("Match Info/Shift Name", "Endgame"); if (time <= 1.0) gameEnded = true; return time; } // endgame
        } else { DogLog.log("Match Info/Shift Name", "Inactive"); return 0.0; }
    }
}

