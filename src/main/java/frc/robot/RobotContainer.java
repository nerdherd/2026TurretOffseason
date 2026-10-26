package frc.robot;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.system.RobotController;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import dev.doglog.DogLog;

import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.math.geometry.Translation2d;

import java.util.NoSuchElementException;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.Subsystems;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.commands.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.SuperSystem.HoodMode;
import frc.robot.subsystems.SuperSystem.IntakeSlideMode;
import frc.robot.subsystems.TurretSwivel.TurretSwivel;
import frc.robot.util.nerd_controller.Controller;
import frc.robot.util.nerd_logging.NerdLog;
import frc.robot.util.nerd_logging.Reportable.LOG_LEVEL;
import frc.robot.util.nerd_math.NerdyMath;


public class RobotContainer {
    public NerdDrivetrain swerveDrive;
    public SuperSystem superSystem;
    public PowerDistribution pdp = new PowerDistribution(CANPort.CAN_D0, 1, ModuleType.REV);

    private final Controller driverController = new Controller(ControllerConstants.kDriverControllerPort);
    private final Controller operatorController = new Controller(ControllerConstants.kOperatorControllerPort);
    private final Controller testController = new Controller(ControllerConstants.kTestControllerPort);

    private static boolean isRedSide = false;

    public RobotContainer() {
        swerveDrive = TunerConstants.createDrivetrain();

        if (Constants.USE_SUBSYSTEMS) {
            superSystem = new SuperSystem(swerveDrive);
        }

        Subsystems.init(); // required to initialize the class or else java lazy loading just doesn't
        Autos.initBindings(superSystem);
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
                () -> new Translation2d(
                    (((driverController.getDpadUp() && !driverController.getBumperRight()) ? 1 : 0) - (driverController.getDpadDown() ? 1 : 0)) * SwerveDriveConstants.kRobotOrientedVelocity, 
                    ((driverController.getDpadLeft() ? 1 : 0) - (driverController.getDpadRight() ? 1 : 0)) * 1.5),
                // joystick drive field oriented
                () -> true, 
                // tow supplier
                () -> driverController.getBumperLeft(), 
                // precision/programmer mode :)
                () -> operatorController.getTriggerRight() ? 1.0 : 0.0
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
       driverController.controllerLeft() // Set Drive Heading
        .onTrue(Commands.runOnce(() -> swerveDrive.setRobotHeadingForward()));

       driverController.controllerRight() // Set Pose Heading (pressed)
        .onTrue(Commands.runOnce(() -> swerveDrive.recalibrateGyroMT1()));

        if (Constants.USE_SUBSYSTEMS) { /* bindings for subsystems */
            driverController.triggerRight()
                .whileTrue(superSystem.startIntakingCommand());
            
        }
    }
    
    ///////////////////////
    // Operator bindings
    //////////////////////
    private void configureOperatorBindings_teleop() {
        
        // Add operator controls here
        if (Constants.USE_SUBSYSTEMS) { /* bindings for subsystems */

            CommandScheduler.getInstance().schedule(
                superSystem.shootCommand(
                    () -> operatorController.getTriggerRight(), // spin up shoot
                    () -> operatorController.getButtonUp()      // spin up pass
                )
            );
            operatorController.bumperLeft()
                .whileTrue(superSystem.startIntakingCommand());
            operatorController.buttonRight()
                .whileTrue(superSystem.startOuttakingCommand());
            operatorController.controllerLeft()
                .onTrue(superSystem.setIntakeSlideCommand(IntakeSlideMode.HOLD));
            operatorController.controllerRight()
                .onTrue(superSystem.setIntakeSlideCommand(IntakeSlideMode.IN))
                .onFalse(superSystem.setIntakeSlideCommand(IntakeSlideMode.STOP));
            operatorController.buttonLeft()
                .onTrue(Subsystems.shooter.setDesiredValueCommand(45))
                .onFalse(superSystem.stopFlywheelCommand());
            operatorController.triggerLeft()
                .onTrue(Subsystems.shooter.setDesiredValueCommand(37))
                .onFalse(superSystem.stopFlywheelCommand());

            operatorController.dpadUp()
                .onTrue(superSystem.setHoodCommand(HoodMode.HIGH))
                .onFalse(superSystem.setHoodCommand(HoodMode.LOW));
            operatorController.dpadDown()
                .onTrue(superSystem.setHoodCommand(HoodMode.MID))
                .onFalse(superSystem.setHoodCommand(HoodMode.LOW));
        }
        
    }

    public void configureBindings_test() {
        Controller.configureDebugBindings(testController);
        TunableDouble speedTuner = TunableDouble.create();

        Tunables.publish("Test/speed", speedTuner);
        operatorController.triggerRight()
            .whileTrue(Commands.run(() -> Subsystems.shooter.setDesiredValue(NerdyMath.clamp(speedTuner.get(), 0.0, 55.0)), Subsystems.shooter));
        
        TunableDouble hoodTuner = TunableDouble.create();
        Tunables.publish("Test/hood", hoodTuner);
        operatorController.triggerLeft()
            .whileTrue(Commands.run(() -> Subsystems.hood.setDesiredValue(NerdyMath.clamp(hoodTuner.get(), HoodConstants.kDownPos, HoodConstants.kUpPos)), Subsystems.hood));
        
        TunableDouble swivelTuner = TunableDouble.create();
        Tunables.publish("Test/swivel", swivelTuner);
        operatorController.buttonRight()
            .whileTrue(Commands.run(() -> Subsystems.turretSwivel.goToAngle(swivelTuner.get()), Subsystems.turretSwivel));
        operatorController.buttonLeft()
            .whileTrue(Commands.run(() -> Subsystems.turretSwivel.goToAngle(TurretSwivel.getRobotRelativeAngle(swerveDrive.getSwerveHeadingDegrees(), swivelTuner.get())), Subsystems.turretSwivel));

        operatorController.bumperRight()
            .whileTrue(superSystem.startIndexingCommand());
        operatorController.bumperLeft()
            .whileTrue(superSystem.lookAtPointCommand(FieldPositions.HUB_CENTER.get()));
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

    public Tunable<String> printLog = null;
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
            "Robot/RAM Available", 
            () -> (double)Runtime.getRuntime().freeMemory(), 
            LOG_LEVEL.ALL);
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
        // if (!RobotState.isFMSAttached()) { DogLog.forceNT.log("Match Info/Shift Name", "DriverStation not attached"); return 0.0; };
        boolean wonAuto = true;
        if (Constants.ROBOT_LOG_LEVEL == LOG_LEVEL.MEDIUM) {
            try {
                String data = MatchState.getGameData().get();
                if (!data.isEmpty()) switch (data.charAt(0)) {
                    case 'B': wonAuto = !isRedSide; break;
                    case 'R': wonAuto = isRedSide; break;
                    default: break;
                } 
            } catch (NoSuchElementException e) {}
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

