package frc.robot.subsystems.TurretSwivel;

import static frc.robot.Constants.LoggingConstants.kSubsystemTab;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import dev.doglog.DogLog;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.nerd_math.NerdyMath;

public class TurretSwivel extends TemplateSubsystem {
    private TurretSwivelConfiguration turretSwivelConfiguration;
    private double targetPos = 0.0;

    /**
     * Initializes a turret swivel
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#TemplateSubsystem(String, int, SubsystemMode, double, boolean) TemplateSubsystem}
     */
    public TurretSwivel(String name, int motorID, SubsystemMode mode, double defaultValue, boolean useSubsystem){
        super(name, motorID, mode, defaultValue, useSubsystem);
        this.targetPos = defaultValue;
    }

    /**
     * Set the configuration for this turret swivel
     * @param configuration the {@link frc.robot.subsystems.TurretSwivel.TurretSwivelConfiguration TurretSwivelConfiguration} to use
     * @return this
     */
    public TurretSwivel setTurretSwivelConfiguration(TurretSwivelConfiguration configuration) {
        turretSwivelConfiguration = configuration;
        return this;
    }

    /**
     * Get the angle for the turret in robot relative space given the robot heading in world space and the desired angle in world space
     * <p>
     * Essentially just turret angle (in world space) - robot heading
     * Any units, they just need to match
     * @param robotHeading Heading of the robot in world space, degrees
     * @param desiredAngle Desired angle of the turret in world space
     * @return Necessary angle for the turret in robot relative space
     */
    public static double getRobotRelativeAngle(double robotHeading, double desiredAngle) {
        // "turret space" is clockwise, but world space is counterclockwise
        return robotHeading - desiredAngle;
    }

    /**
     * Checks if the turret is currently ready to shoot
     * <p>
     * Takes into account current turret rotation and current turret velocity
     * @return Whether or not the turret is ready to shoot or not
     */
    public boolean isReadyToShoot(NerdDrivetrain drivetrain){
        if (Math.abs(getRobotRelativeAngle(drivetrain.getRotationalVelocity(), this.getCurrentVelocity()*Math.TAU)) > turretSwivelConfiguration.maxVelocityTolerance())
            return false;
        if (isValueWithinTolerance(turretSwivelConfiguration.maxAngleTolerance()))
            return false;
        // double angleDegrees = this.getCurrentPosition()*360;
        if (targetPos < turretSwivelConfiguration.lowerAngleBound() || targetPos > turretSwivelConfiguration.upperAngleBound())
            return false;
        return true;
    }

    /**
     * Rotates the turret to face a certain angle
     * <p>
     * Will not turn if desired angle is within deadband
     * @param desiredAngle Desired angle in degrees, robot relative
     */
    public void goToAngle(double desiredAngle){
        desiredAngle = NerdyMath.posMod(desiredAngle, 360);
        targetPos = desiredAngle;
        if (desiredAngle < turretSwivelConfiguration.lowerAngleBound() || desiredAngle > turretSwivelConfiguration.upperAngleBound())
            return;
        this.setDesiredValue(desiredAngle/360.0);
    }

    /**
     * Calls {@link #goToAngle(double)}
     * @param desiredAngle Desired angle in degrees, robot relative
     * @return command, runOnce
     */
    public Command goToAngleCommand(double desiredAngle) {
        return Commands.runOnce(() -> goToAngle(desiredAngle));
    }
    
    /**
     * Given a point in robot space, return that point rotated around the turret origin by the current heading
     * @param original The original point
     * @return The rotated point
     */
    public Pose3d getRotatedPose3d(Pose3d original) {
        return original.rotateAround(
            new Translation3d(turretSwivelConfiguration.relativePosition().getTranslation()), 
            new Rotation3d(Rotation2d.fromRotations(getCurrentPosition()))
        );
    }

    /**
     * Gets the relative position of the turret to the center of the robot
     * @return The relative position of the turret to the center of the robot
     */
    public Pose2d getRelativePose(){
        return turretSwivelConfiguration.relativePosition();
    }

    /** 
     * {@inheritDoc}
     * <p>
     * Overrides {@link frc.robot.subsystems.template.TemplateSubsystem#configureMotors(TalonFXConfiguration) TemplateSubsystem.configureMotors} to return TurretSwivel
     * <p>
     * Otherwise calling this function would return a TemplateSubsystem, which could potentially lose data when cast
     * <p>
     * Does exactly the same thing as the function it overrides
     * <p>
     */
    @Override
	public TurretSwivel configureMotors(TalonFXConfiguration configuration){
		this.configuration = configuration;
		DogLog.log(kSubsystemTab + name + "/motor configs", configuration.toString());
		applyMotorConfigs();
		return this;
	}
}
