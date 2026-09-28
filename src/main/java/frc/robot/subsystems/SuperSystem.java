package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ConveyorBeltConstants;
import frc.robot.Constants.ConveyorRollerConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.Constants.IntakeSlideConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.nerd_logging.NerdLog;
import frc.robot.util.nerd_logging.Reportable;
import frc.robot.util.nerd_math.NerdyMath;

import static frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import static frc.robot.Constants.Subsystems.intakeSlide;
import static frc.robot.Constants.Subsystems.intakeRoller;
import static frc.robot.Constants.Subsystems.indexer;
import static frc.robot.Constants.Subsystems.conveyorBelt;
import static frc.robot.Constants.Subsystems.conveyorRoller;
import static frc.robot.Constants.Subsystems.shooter;
import static frc.robot.Constants.Subsystems.turretSwivel;
import static frc.robot.Constants.Subsystems.hood;


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

    public Command intakeOutOnly() {
        return intakeSlide.setDesiredValueCommand(IntakeSlideConstants.kOutVoltage); // test actual number
    }

    public Command intakeHold() {
        return intakeSlide.setDesiredValueCommand(IntakeSlideConstants.kHoldVoltage); 
    }

    public Command stopIntakeHold(){
        return intakeSlide.setDesiredValueCommand(0);
    }
    
    public Command intake() {
        return Commands.parallel(
            intakeRoller.setDesiredValueCommand(IntakeRollerConstants.kIntakeVoltage),
            stopIntakeHold()
        );
    }

    public Command outtake(){
        return intakeRoller.setDesiredValueCommand(IntakeRollerConstants.kOuttakeVoltage);
    }

    public Command stopIntaking() {
        return Commands.parallel (
        intakeRoller.setDesiredValueCommand(0),
        stopIntakeHold()
        );
    } 

    public Command spinConveyorForward() {
        return Commands.parallel(
            conveyorRoller.setDesiredValueCommand(ConveyorRollerConstants.kConveyorVoltage),
            conveyorBelt.setDesiredValueCommand(ConveyorBeltConstants.kConveyorVoltage),
            indexer.setDesiredValueCommand(IndexerConstants.kConveyorVoltage)
        );
    }
    
    public Command stopConveyor() {
        return Commands.parallel(
            conveyorRoller.setDesiredValueCommand(0),
            conveyorBelt.setDesiredValueCommand(0),
            indexer.setDesiredValueCommand(0)
        );
    }
    
    public Command spinConveyorBackward() {
        return Commands.parallel(
            conveyorRoller.setDesiredValueCommand(-ConveyorRollerConstants.kConveyorVoltage),
            conveyorBelt.setDesiredValueCommand(-ConveyorBeltConstants.kConveyorVoltage),
            indexer.setDesiredValueCommand(-IndexerConstants.kConveyorVoltage)
        );
    }
    
    public Command spinUpFlywheel() {
        return shooter.setDesiredValueCommand(ShooterConstants.kShootVelocity);
    }
    
    
    public Command spinUpFlywheelFeeding() {
        return shooter.setDesiredValueCommand(ShooterConstants.kFeedingVelocity);
    }
    
    public Command stopFlywheel() {
        return shooter.setDesiredValueCommand(0);
    }

    public Command setHood(double value) {
        value = (HoodConstants.kUpPos-HoodConstants.kDownPos) * value + HoodConstants.kDownPos;
        return hood.setDesiredValueCommand(value);
    }

    public Command hoodDown() {
        return setHood(0.0);
    }
    
    public Command hoodUp() {
        return setHood(1.0);
    }

    // public Command setTurretSwivel(){
    //     double angle = NerdyMath.angleToPose(swerveDrivetrain.getLookAheadPose(ShooterConstants.kLookAheadFactor), FieldPositions.HUB_CENTER.get());
    //     double value = 360-angle;
    //     return turretSwivel.goToAngleCommand(value);
    // }

    /**
     * Attempt to rotate the turret to look at the current hub, with look ahead.
     * Rotational speed is added to the expected robot rotation.
     */
    public void lookAtHub(){ // simulated, not tested
        Pose2d expectedRobotPose = swerveDrivetrain.getLookAheadPoseWithRotation(ShooterConstants.kLookAheadFactor);
        Pose2d expectedTurretPose = expectedRobotPose.transformBy(new Transform2d(turretSwivel.getRelativePose().getTranslation(), Rotation2d.ZERO));
        double angleToHubRad = NerdyMath.angleToPose(expectedTurretPose, FieldPositions.HUB_CENTER.get()) - expectedRobotPose.getRotation().getRadians();

        turretSwivel.goToAngle(NerdyMath.radiansToDegrees(angleToHubRad));

        // TODO: Comment this line when not simulating
        Telemetry.log("Turret Pose", new Pose2d(expectedTurretPose.getTranslation(), new Rotation2d(angleToHubRad + expectedRobotPose.getRotation().getRadians())));
    }

    /**
     * Attempt to rotate the turret to look at the current hub, with look ahead.
     * Rotational speed is converted to translational speed and added to the expected turret position.
     */
    public void lookAtHubMason(){ // simulated, not tested
        // gets the pose of the robot translated by its velocity times a factor, but not changing its rotation
        Pose2d expectedRobotPose = swerveDrivetrain.getLookAheadPose(ShooterConstants.kLookAheadFactor);

        double robotAngularVelocity = swerveDrivetrain.getRotationalSpeed();

        // gets the turret's location relative to the robot's center, but rotated to match field space
        Pose2d turretOffset = new Pose2d(turretSwivel.getRelativePose().rotateBy(expectedRobotPose.getRotation()).getTranslation(), Rotation2d.ZERO);

        // creates a point 90 degrees counterclockwise from the robot's center to turretOffset 
        Pose2d turretSpeedVector = new Pose2d(-turretOffset.getY(),turretOffset.getX(),Rotation2d.ZERO);

        // sets the magnitude of turretSpeedVector (as in its distance from the origin) based on the robot's angular velocity times the factor
        // in other words, creates a vector representing the turret's velocity in field space.
        turretSpeedVector = turretSpeedVector.times(robotAngularVelocity).times(ShooterConstants.kLookAheadFactor);

        // offsets the turret position by the turret speed vector to create its expected position
        Pose2d expectedTurretPose = turretOffset
            .plus(new Transform2d(expectedRobotPose.getTranslation(),Rotation2d.ZERO))
            .plus(new Transform2d(turretSpeedVector.getTranslation(),Rotation2d.ZERO));

        // the angle from the expected turret pose to the hub
        double angleToHubRad = NerdyMath.angleToPose(expectedTurretPose, FieldPositions.HUB_CENTER.get()) - expectedRobotPose.getRotation().getRadians();

        turretSwivel.goToAngle(NerdyMath.radiansToDegrees(angleToHubRad));

        // TODO: Comment this line when not simulating
        Telemetry.log("Turret Pose", new Pose2d(expectedTurretPose.getTranslation(), new Rotation2d(angleToHubRad + expectedRobotPose.getRotation().getRadians())));
    }

    public Command lookAtHubCommand(){
        return Commands.runOnce(() -> lookAtHub());
    }

    public Command lookAtHubMasonCommand(){
        return Commands.runOnce(() -> lookAtHubMason());
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
        NerdLog.logData(LoggingConstants.kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    }

      // ------------------------------------ subsystems ------------------------------------ //
    


    
}
