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
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.util.nerd_logging.NerdLog;
import frc.robot.util.nerd_math.NerdyMath;

import static frc.robot.Constants.SwerveDriveConstants.FieldPositions;

import java.util.function.Supplier;

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
    public void setIntakeSlide(IntakeSlideMode mode) {
        switch(mode) {
            case OUT:       intakeSlide.setDesiredValue(-3); break;
            case IN:        intakeSlide.setDesiredValue(3); break;
            case HOLD:      intakeSlide.setDesiredValue(-1); break;
            case STOP: default: intakeSlide.setDesiredValue(0); break;
        }
    }
    public Command setIntakeSlideCommand(IntakeSlideMode mode) {
        return Commands.runOnce(() -> setIntakeSlide(mode), intakeSlide);
    }
    
    public enum IntakeRollerMode{INTAKE, OUTTAKE, STOP};
    public void setIntakeRoller(IntakeRollerMode mode) {
        switch(mode) {
            case INTAKE:    intakeRoller.setDesiredValue(8); break;
            case OUTTAKE:   intakeRoller.setDesiredValue(-8); break;
            case STOP: default: intakeRoller.setDesiredValue(0); break;
        }
    }
    public Command setIntakeRollerCommand(IntakeRollerMode mode) {
        return Commands.runOnce(() -> setIntakeRoller(mode), intakeRoller);
    }

    public enum RollerFloorMode {IN, OUT, STOP};
    public void setRollerFloor(RollerFloorMode mode) {
        switch(mode) {
            case IN:        rollerFloor.setDesiredValue(8); break;
            case OUT:       rollerFloor.setDesiredValue(-8); break;
            case STOP: default: rollerFloor.setDesiredValue(0); break;
        }
    }
    public Command setRollerFloorCommand(RollerFloorMode mode) {
        return Commands.runOnce(() -> setRollerFloor(mode), rollerFloor);
    }

    public enum ConveyorBeltMode {IN, OUT, STOP};
    public void setConveyorBelt(ConveyorBeltMode mode) {
        switch(mode) {
            case IN:        conveyorBelt.setDesiredValue(8); break;
            case OUT:       conveyorBelt.setDesiredValue(-8); break;
            case STOP: default: conveyorBelt.setDesiredValue(0); break;
        }
    }
    public Command setConveyorBeltCommand(ConveyorBeltMode mode) {
        return Commands.runOnce(() -> setConveyorBelt(mode), conveyorBelt);
    }

    public enum ThroatMode {IN, OUT, STOP};
    public void setThroat(ThroatMode mode) {
        switch(mode) {
            case IN:        throat.setDesiredValue(8); break;
            case OUT:       throat.setDesiredValue(-8); break;
            case STOP: default: throat.setDesiredValue(0); break;
        }
    }
    public Command setThroatCommand(ThroatMode mode) {
        return Commands.runOnce(() -> setThroat(mode), throat);
    }

    public enum HoodMode {HIGH, MID, LOW};
    public void setHood(HoodMode mode) {
        switch(mode) {
            case HIGH:      hood.setDesiredValue(HoodConstants.kUpPos); break;
            case MID:       hood.setDesiredValue((HoodConstants.kUpPos + HoodConstants.kDownPos) * 0.5); break;
            case LOW: default: hood.setDesiredValue(HoodConstants.kDownPos); break;
        }
    }
    public Command setHoodCommand(HoodMode mode) {
        return Commands.runOnce(() -> setHood(mode), throat);
    }
    
    // ------------------------------------ game actions ------------------------------------ //
    /** @return continuous */
    public Command startIntakingCommand() {
        return continuousParallelCommand(
            setIntakeSlideCommand(IntakeSlideMode.HOLD),
            setIntakeRollerCommand(IntakeRollerMode.INTAKE)
        ).andThen(stopIntakeCommand());
    }
    
    /** @return continuous */
    public Command startOuttakingCommand(){
        return continuousParallelCommand(
                setIntakeSlideCommand(IntakeSlideMode.HOLD),
                setIntakeRollerCommand(IntakeRollerMode.OUTTAKE),
                setRollerFloorCommand(RollerFloorMode.OUT),
                setConveyorBeltCommand(ConveyorBeltMode.OUT),
                setThroatCommand(ThroatMode.OUT)
            ).andThen(stopIntakeCommand());
    }

    /** @return instant */
    public Command stopIntakeCommand() {
        return Commands.parallel(
            setIntakeSlideCommand(IntakeSlideMode.STOP),
            setIntakeRollerCommand(IntakeRollerMode.STOP)
        );
    }

    /** @return continuous */
    public void startIndexing() {
        setRollerFloor(RollerFloorMode.IN);
        setConveyorBelt(ConveyorBeltMode.IN);
        setThroat(ThroatMode.IN);
        // agitate
    }
    public Command startIndexingCommand() {
        return Commands.run(() -> startIndexing(), rollerFloor, conveyorBelt, throat)
            .andThen(stopIndexingCommand());
    }

    /** @return instant */
    public Command stopIndexingCommand() {
        return Commands.parallel(
            setRollerFloorCommand(RollerFloorMode.STOP),
            setConveyorBeltCommand(ConveyorBeltMode.STOP),
            setIntakeSlideCommand(IntakeSlideMode.HOLD)
        );
    }

    /**
     * schedule during teleop
     * @param ejectBinding
     * @param shootBinding
     * @param passBinding
     * @return continuous
     */
    public Command shootCommand(Supplier<Boolean> ejectBinding, Supplier<Boolean> shootBinding, Supplier<Boolean> passBinding) {
        return Commands.run(() -> {
            if (shootBinding.get()) {
                // set flywheel using 
                // point at
                // prepTurret(hubpose)
                if (ejectBinding.get()) {

                    ; // set hood
                }
            } else if (passBinding.get()) {
                // set flywheel 
                // point at
                // prepTurret(passing poses)
                if (ejectBinding.get()) 
                    ; // set hood
            }
            if (ejectBinding.get()) startIndexing();
        }, shooter, hood)
            .andThen(Commands.parallel(
                stopIndexingCommand(),
                stopFlywheelCommand()
            ));
    }

    /** @return instant */
    public Command stopFlywheelCommand() {
        return Commands.runOnce(() -> {
            shooter.setDesiredValue(0.0);
        });
    }

    // TODO: move this
    // TODO: give these better names
    public enum TurretLookAheadMode {ANGLE,TRANSLATION};
    /**
     * 
     * @param mode the mode to use. ANGLE rotates the robot, TRANSLATION calculates the translational velocity
     * @return A Pose2d. The x and y are the position of the turret in field space. The rotation is the rotation of the robot
     */
    public Pose2d getExpectedTurretPosition(TurretLookAheadMode mode) {
        switch (mode) {
            case ANGLE -> {
                // Rotational speed is added to the expected robot rotation
                Pose2d expectedRobotPose = swerveDrivetrain.getLookAheadPoseWithRotation(ShooterConstants.kLookAheadFactor);
                return expectedRobotPose.transformBy(new Transform2d(turretSwivel.getRelativePose().getTranslation(), expectedRobotPose.getRotation()));
            }
            case TRANSLATION -> {
                // Rotational speed is converted to translational speed and added to the expected turret position.

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

                Pose2d expectedTurretPosition = turretOffset
                    .plus(new Transform2d(expectedRobotPose.getTranslation(),Rotation2d.ZERO))
                    .plus(new Transform2d(turretSpeedVector.getTranslation(),Rotation2d.ZERO));
                // offsets the turret position by the turret speed vector to create its expected position
                return new Pose2d(expectedTurretPosition.getTranslation(),expectedTurretPosition.getRotation());
            }
        }
        return Pose2d.ZERO;
    }

    // TODO: test this
    public void lookAtHub(TurretLookAheadMode mode) {
        Pose2d expectedTurretPose = getExpectedTurretPosition(mode);

        double angleToHubRad = NerdyMath.angleToPose(expectedTurretPose, FieldPositions.HUB_CENTER.get()) - expectedTurretPose.getRotation().getRadians();

        turretSwivel.goToAngle(NerdyMath.radiansToDegrees(angleToHubRad));

        // TODO: Comment this line when not simulating
        Telemetry.log("Turret Pose", new Pose2d(expectedTurretPose.getTranslation(), Rotation2d.fromDegrees(turretSwivel.getDesiredValue()*360 + expectedRobotPose.getRotation().getDegrees())));
    }

    /**
     * @param mode
     * @return continuous
     */
    public Command lookAtHubCommand(TurretLookAheadMode mode){
        return Commands.run(() -> lookAtHub(mode));
    }

    // ------------------------------------ logging ------------------------------------ //
    @Override
    public void initializeLogging() {
        super.initializeLogging();
        NerdLog.logData(LoggingConstants.kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    }
}
