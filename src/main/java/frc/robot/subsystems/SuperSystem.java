package frc.robot.subsystems;

import static frc.robot.Constants.Subsystems.conveyorBelt;
import static frc.robot.Constants.Subsystems.hood;
import static frc.robot.Constants.Subsystems.intakeRoller;
import static frc.robot.Constants.Subsystems.intakeSlide;
import static frc.robot.Constants.Subsystems.rollerFloor;
import static frc.robot.Constants.Subsystems.shooter;
import static frc.robot.Constants.Subsystems.throat;
import static frc.robot.Constants.Subsystems.turretSwivel;

import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.util.MathSharedStore;
import org.wpilib.telemetry.Telemetry;

import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.subsystems.TurretSwivel.TurretSwivel;
import frc.robot.util.nerd_logging.NerdLog;
import frc.robot.util.nerd_math.NerdyMath;

/**
 * Coordinates subsystems
 * <p>
 * Provides commands to be bound to controller buttons that run individual subsystems as needed
 */
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
    /**
     * @param mode mode to use
     * @return instant
     * @see {@link #setIntakeSlide(IntakeSlideMode)}
     */
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
    /**
     * @param mode mode to use
     * @return instant
     * @see {@link #setIntakeRoller(IntakeRollerMode)}
     */
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
    /**
     * @param mode mode to use
     * @return instant
     * @see {@link #setRollerFloor(RollerFloorMode)}
     */
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
    /**
     * @param mode mode to use
     * @return instant
     * @see {@link #setConveyorBelt(ConveyorBeltMode)}
     */
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
    /**
     * @param mode mode to use
     * @return instant
     * @see {@link #setThroat(ThroatMode)}
     */
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
    /**
     * @param mode mode to use
     * @return instant
     * @see {@link #setHood(HoodMode)}
     */
    public Command setHoodCommand(HoodMode mode) {
        return Commands.runOnce(() -> setHood(mode), hood);
    }
    
    // ------------------------------------ game actions ------------------------------------ //
    /** 
     * Holds slide, spins intake roller
     * @return continuous 
     * @see {@link #setIntakeSlide(IntakeSlideMode)} {@link IntakeSlideMode#HOLD}
     * @see {@link #setIntakeRoller(IntakeRollerMode)} {@link IntakeRollerMode#INTAKE}
     */
    public Command startIntakingCommand() {
        return continuousParallelCommand(
            setIntakeSlideCommand(IntakeSlideMode.HOLD),
            setIntakeRollerCommand(IntakeRollerMode.INTAKE)
        ).andThen(stopIntakeCommand());
    }
    
    /** 
     * Holds slide, reverses intake, floor, belt, and throat to outtake fuel
     * @return continuous 
     * @see {@link #setIntakeSlide(IntakeSlideMode)} {@link IntakeSlideMode#HOLD}
     * @see {@link #setIntakeRoller(IntakeRollerMode)} {@link IntakeRollerMode#OUTTAKE}
     * @see {@link #setRollerFloor(RollerFloorMode)} {@link RollerFloorMode#OUT}
     * @see {@link #setConveyorBelt(ConveyorBeltMode)} {@link ConveyorBeltMode#OUT}
     * @see {@link #setThroat(ThroatMode)} {@link ThroatMode#OUT}
     */
    public Command startOuttakingCommand(){
        return continuousParallelCommand(
                setIntakeSlideCommand(IntakeSlideMode.HOLD),
                setIntakeRollerCommand(IntakeRollerMode.OUTTAKE),
                setRollerFloorCommand(RollerFloorMode.OUT),
                setConveyorBeltCommand(ConveyorBeltMode.OUT),
                setThroatCommand(ThroatMode.OUT)
            ).andThen(stopIntakeCommand());
    }

    /** 
     * Stops intake, stops slide
     * @return instant 
     * @see {@link #setIntakeSlide(IntakeSlideMode)} {@link IntakeSlideMode#STOP}
     * @see {@link #setIntakeRoller(IntakeRollerMode)} {@link IntakeRollerMode#STOP}
     */
    public Command stopIntakeCommand() {
        return Commands.parallel(
            setIntakeSlideCommand(IntakeSlideMode.STOP),
            setIntakeRollerCommand(IntakeRollerMode.STOP)
        );
    }

    /**
     * 
     * COMPRESS has the intake slide go inwards at a low voltage.
     * JIMMY has the intake move in and out
     */
    public enum AgitateMode {COMPRESS, JIMMY}

    private double startShootTime = 0.0;

    /** 
     * Spins the roller floor, conveyor belt, and throat inwards
     * @param mode The AgitateMode to use when indexing
     * @see {@link #setRollerFloor(RollerFloorMode)} {@link RollerFloorMode#IN}
     * @see {@link #setConveyorBelt(ConveyorBeltMode)} {@link ConveyorBeltMode#IN}
     * @see {@link #setThroat(ThroatMode)} {@link ThroatMode#IN}
     * @see {@link AgitateMode}
     */
    public void startIndexing(AgitateMode mode) {
        setRollerFloor(RollerFloorMode.IN);
        setConveyorBelt(ConveyorBeltMode.IN);
        setThroat(ThroatMode.IN);
        switch(mode){
            case COMPRESS -> {
                setIntakeSlide(IntakeSlideMode.IN);
            }
            case JIMMY -> {
                double val = NerdyMath.posMod(MathSharedStore.getTimestamp() - startShootTime, 0.7);
                if (val <= 0.5) setIntakeSlide(IntakeSlideMode.IN);
                else if (val <= 0.7) setIntakeSlide(IntakeSlideMode.OUT);
                else setIntakeSlide(IntakeSlideMode.STOP);
            }
        }
        
    }

    /** 
     * Starts indexing, then stops indexing after finished
     * @return continuous 
     * @see {@link #startIndexing()}
     * @see {@link #stopIndexingCommand()}
     */
    public Command startIndexingCommand() {
        return Commands.runOnce(()->startShootTime = MathSharedStore.getTimestamp()).andThen(
            Commands.run(() -> startIndexing(AgitateMode.COMPRESS), rollerFloor, conveyorBelt, throat)
            .andThen(stopIndexingCommand())
        );
    }

    /** 
     * Stops the roller floor, conveyor belt, and intake slide
     * @return instant 
     * @see {@link #setRollerFloor(RollerFloorMode)} {@link RollerFloorMode#STOP}
     * @see {@link #setConveyorBelt(ConveyorBeltMode)} {@link ConveyorBeltMode#STOP}
     * @see {@link #setIntakeSlide(IntakeSlideMode)} {@link IntakeSlideMode#STOP}
     */
    public Command stopIndexingCommand() {
        return Commands.parallel(
            setRollerFloorCommand(RollerFloorMode.STOP),
            setConveyorBeltCommand(ConveyorBeltMode.STOP),
            setIntakeSlideCommand(IntakeSlideMode.STOP)
        );
    }

    /**
     * schedule during teleop
     * TODO: Javadoc
     * @param ejectBinding
     * @param shootBinding
     * @param passBinding
     * @return continuous
     */
    public Command shootCommand(Supplier<Boolean> ejectBinding, Supplier<Boolean> shootBinding, Supplier<Boolean> passBinding) {
        return Commands.run(() -> {
            if (shootBinding.get()) {
                prepTurret(FieldPositions.HUB_CENTER.get());
                if (ejectBinding.get()) {

                    ; // set hood
                }
            } else if (passBinding.get()) {
                // prepTurret();
                if (ejectBinding.get()) 
                    ; // set hood
            }
            if (ejectBinding.get()) startIndexing(AgitateMode.COMPRESS);
        }, shooter, hood, turretSwivel, rollerFloor, throat)
            .andThen(Commands.parallel(
                stopIndexingCommand(),
                stopFlywheelCommand()
            ));
    }

    /** 
     * set the flywheel velocity to 0
     * @return instant
     */
    public Command stopFlywheelCommand() {
        return Commands.runOnce(() -> {
            shooter.setDesiredValue(0.0);
        }, shooter);
    }

    /**
     * Both turns the turret to the target and spins up the flywheel
     * @param target
     * @see {@link #lookAtPoint(Pose2d)}
     * @see {@link #shootWithDistance(Pose2d)}
     */
    public void prepTurret(Pose2d target) {
        lookAtPoint(target);
        shootWithDistance(target);
    }

    /** 
     * Both turns the turret to the target and spins up the flywheel
     * <p>
     * Stops the flywheel on end
     * @return continuous 
     * @see {@link #prepTurret(Pose2d)}
     */
    public Command prepTurretCommand(Pose2d target) {
        return Commands.run(() -> prepTurret(target), turretSwivel, shooter, hood)
            .andThen(stopFlywheelCommand());
    }

    /**
     * Spins the flywheel at a speed depending on the distance to a target
     * <p>
     * Assumes turret is already pointing toward the target
     * <p>
     * Assumes shooting at height of the hub
     * @param target The target to shoot at
     * @see {@link #getTurretDistanceTo(Pose2d)}
     * @see {@link frc.robot.Constants.ShooterConstants#kShooterTable ShooterConstants.kShooterTable}
     */
    public void shootWithDistance(Pose2d target) {
        double distance = getTurretDistanceTo(target);
        // TODO call turret is ready to shoot
        shooter.setDesiredValue(ShooterConstants.kShooterTable.interpolate(distance));
    }

    /** 
     * Runs shootWithDistance, then stops the flywheel on end
     * @return continuous 
     * @see {@link #shootWithDistance(Pose2d)}
     */
    public Command shootWithDistanceCommand(Pose2d target) {
        return Commands.run(() -> shootWithDistance(target), shooter)
            .andThen(stopFlywheelCommand());
    }

    /**
     * Turns the turret to look at a specific point on the field
     * @param point the location to turn to, in field space
     * @see {@link #getExpectedTurretPosition()}
     */
    public void lookAtPoint(Pose2d point) {
        Pose2d expectedTurretPose = getExpectedTurretPosition();

        double angleToHubRad = TurretSwivel.getRobotRelativeAngle(
            expectedTurretPose.getRotation().getRadians(), // actually expected robot rotation
            NerdyMath.angleToPose(expectedTurretPose, point)
        );
        
        turretSwivel.goToAngle(NerdyMath.radiansToDegrees(angleToHubRad));

        if (RobotBase.isSimulation()) Telemetry.log("Turret Pose", new Pose2d(expectedTurretPose.getTranslation(), Rotation2d.fromDegrees(turretSwivel.getDesiredValue()*360 + expectedTurretPose.getRotation().getDegrees())));
    }

    /** 
     * @return continuous 
     * @see {@link #lookAtPoint(Pose2d)}
     */
    public Command lookAtPointCommand(Pose2d point) {
        return Commands.run(() -> lookAtPoint(point), turretSwivel);
    }

    /** 
     * @return continuous 
     * @see {@link #lookAtPointCommand(Pose2d)}
     */
    public Command lookAtHubCommand() {
        return lookAtPointCommand(FieldPositions.HUB_CENTER.get());
    }

    
    // ------------------------------------ helper functions ------------------------------------ //

    public enum TurretLookAheadMode {
        /**
         * Offset the robot's rotation by the angular velocity
         */
        ANGLE,
        /**
         * Offset the turret's position by the tangential velocity
         */
        TRANSLATION
    };
    /**
     * Calculate the turret position, plus a little lookahead
     * @return A Pose2d. The x and y are the expected position of the turret in field space. The rotation is the expected rotation of the robot
     */
    public Pose2d getExpectedTurretPosition() {
        TurretLookAheadMode mode = TurretLookAheadMode.ANGLE;
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
                return new Pose2d(expectedTurretPosition.getTranslation(),expectedRobotPose.getRotation());
            }
        }
        return Pose2d.ZERO;
    }

    /**
     * Gets the distance between the turret's expected position and a target
     * @param target the target to calculate the distance to
     * @return the distance between the turret's expected position and the target
     * @see {@link #getExpectedTurretPosition()}
     */
    public double getTurretDistanceTo(Pose2d target) {
        return getExpectedTurretPosition().getTranslation().getDistance(target.getTranslation());
    }

    // ------------------------------------ logging ------------------------------------ //
    /**
     * {@inheritDoc}
     */
    @Override
    public void initializeLogging() {
        super.initializeLogging();
        NerdLog.logData(LoggingConstants.kSupersystemTab + "/Command Scheduler", CommandScheduler.getInstance(), LOG_LEVEL.ALL);
    }
}
