package frc.robot.subsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.telemetry.Telemetry;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.nerd_logging.NerdLog;
import frc.robot.util.nerd_math.NerdyMath;

import static frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import static frc.robot.Constants.Subsystems.intakeSlide;
import static frc.robot.Constants.Subsystems.intakeRoller;
import static frc.robot.Constants.Subsystems.throat;
import static frc.robot.Constants.Subsystems.conveyorBelt;
import static frc.robot.Constants.Subsystems.rollerFloor;
import static frc.robot.Constants.Subsystems.shooter;
import static frc.robot.Constants.Subsystems.turretSwivel;
import static frc.robot.Constants.Subsystems.hood;


public class SuperSystem extends SuperSystemBase {
    public SuperSystem(NerdDrivetrain swerveDrivetrain) {
        super(swerveDrivetrain);
    }

    // ------------------------------------ subsystems ------------------------------------ //
    public enum IntakeSlideMode{OUT, IN, HOLD, STOP};
    public Command intakeSlideCommand(IntakeSlideMode mode) {
        switch(mode) {
            case OUT:   return intakeSlide.setDesiredValueCommand(-3);
            case IN:    return intakeSlide.setDesiredValueCommand(3);
            case HOLD:  return intakeSlide.setDesiredValueCommand(-1);
            case STOP: default: return intakeSlide.setDesiredValueCommand(0);
        }
    }
    
    public enum IntakeRollerMode{INTAKE, OUTTAKE, STOP};
    public Command intakeRollerCommand(IntakeRollerMode mode) {
        switch(mode) {
            case INTAKE:    return intakeRoller.setDesiredValueCommand(8);
            case OUTTAKE:   return intakeRoller.setDesiredValueCommand(-8);
            case STOP: default: return intakeRoller.setDesiredValueCommand(0);
        }
    }

    public enum RollerFloorMode {IN, OUT, STOP};
    public Command rollerFloorCommand(RollerFloorMode mode) {
        switch(mode) {
            case IN:    return rollerFloor.setDesiredValueCommand(8);
            case OUT:   return rollerFloor.setDesiredValueCommand(-8);
            case STOP: default: return rollerFloor.setDesiredValueCommand(0);
        }
    }

    public enum ConveyorBeltMode {IN, OUT, STOP};
    public Command conveyorBeltCommand(ConveyorBeltMode mode) {
        switch(mode) {
            case IN:    return conveyorBelt.setDesiredValueCommand(8);
            case OUT:   return conveyorBelt.setDesiredValueCommand(-8);
            case STOP: default: return conveyorBelt.setDesiredValueCommand(0);
        }
    }

    public enum ThroatMode {IN, OUT, STOP};
    public Command throatCommand(ThroatMode mode) {
        switch(mode) {
            case IN:    return throat.setDesiredValueCommand(8);
            case OUT:   return throat.setDesiredValueCommand(-8);
            case STOP: default: return throat.setDesiredValueCommand(0);
        }
    }
    
    // ------------------------------------ game actions ------------------------------------ //
    /** @return continuous */
    public Command startIntakingCommand() {
        return continuousParallelCommand(
            intakeSlideCommand(IntakeSlideMode.HOLD),
            intakeRollerCommand(IntakeRollerMode.INTAKE)
        ).andThen(stopIntakeCommand());
    }
    
    /** @return continuous */
    public Command startOuttakingCommand(){
        return continuousParallelCommand(
                intakeSlideCommand(IntakeSlideMode.HOLD),
                intakeRollerCommand(IntakeRollerMode.OUTTAKE),
                rollerFloorCommand(RollerFloorMode.OUT),
                conveyorBeltCommand(ConveyorBeltMode.OUT),
                throatCommand(ThroatMode.OUT)
            ).andThen(stopIntakeCommand());
    }

    /** @return instant */
    public Command stopIntakeCommand() {
        return Commands.parallel(
            intakeSlideCommand(IntakeSlideMode.STOP),
            intakeRollerCommand(IntakeRollerMode.STOP)
        );
    }

    /** @return continuous */
    public Command startIndexing() {
        return Commands.parallel(
            rollerFloorCommand(RollerFloorMode.IN),
            conveyorBeltCommand(ConveyorBeltMode.IN),
            throatCommand(ThroatMode.IN),
            Commands.run(() -> {
                // TODO agitate
            }, intakeSlide)
        ).andThen(stopIndexing());
    }

    /** @return instant */
    public Command stopIndexing() {
        return Commands.parallel(
            rollerFloorCommand(RollerFloorMode.STOP),
            conveyorBeltCommand(ConveyorBeltMode.STOP)
        );
    }

    /** @return instant */


    // progress ^^^
    
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
        Telemetry.log("Turret Pose", new Pose2d(expectedTurretPose.getTranslation(), Rotation2d.fromDegrees(turretSwivel.getDesiredValue()*360 + expectedRobotPose.getRotation().getDegrees())));
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
        Telemetry.log("Turret Pose", new Pose2d(expectedTurretPose.getTranslation(), Rotation2d.fromDegrees(turretSwivel.getDesiredValue()*360 + expectedRobotPose.getRotation().getDegrees())));
    }

    public Command lookAtHubCommand(){
        return Commands.run(() -> lookAtHub());
    }

    public Command lookAtHubMasonCommand(){
        return Commands.run(() -> lookAtHubMason());
    }

    // ------------------------------------ logging ------------------------------------ //
    @Override
    public void initializeLogging() {
        super.initializeLogging();
        NerdLog.logData(LoggingConstants.kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    }
}
