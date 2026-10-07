package frc.robot.subsystems.TurretSwivel;

import static frc.robot.Constants.LoggingConstants.kSubsystemTab;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import dev.doglog.DogLog;
import frc.robot.subsystems.NerdDrivetrain;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.nerd_math.NerdyMath;

public class TurretSwivel extends TemplateSubsystem {
    private TurretSwivelConfiguration turretSwivelConfiguration;

    /**
     * Initializes a turret swivel
     * @see {@link frc.robot.subsystems.template.TemplateSubsystem#TemplateSubsystem(String, int, SubsystemMode, double, boolean) TemplateSubsystem}
     */
    public TurretSwivel(String name, int motorID, SubsystemMode mode, double defaultValue, boolean useSubsystem){
        super(name, motorID, mode, defaultValue, useSubsystem);
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

    // TODO: figure out if we need tihs
    // public Pose2d getPose(Pose2d swervePosition, double heading) {
    //     Translation2d translation = turretSwivelConfiguration.relativePosition().getTranslation().rotateBy(Rotation2d.fromDegrees(heading));
    //     return swervePosition.transformBy(new Transform2d(translation, Rotation2d.ZERO));
    // }

    /**
     * Get the angle for the turret in robot relative space given the robot heading in world space and the desired angle in world space
     * <p>
     * {@code robotHeading} and {@code desiredAngle} must be in the same unit
     * @param robotHeading Heading of the robot in world space
     * @param desiredAngle Desired angle of the turret in world space
     * @return Necessary angle for the turret in robot relative space, in the same unit as the inputs
     */
    public static double getRobotRelativeAngle(double robotHeading, double desiredAngle) {
        return desiredAngle - robotHeading;
    }

    /**
     * Checks if the turret is currently ready to shoot
     * <p>
     * Takes into account current turret rotation and current turret velocity
     * @return Whether or not the turret is ready to shoot or not
     */
    public boolean isReadyToShoot(NerdDrivetrain drivetrain){
        if (Math.abs(this.getCurrentVelocity() + (drivetrain.getRotationalVelocity() / (2 * Math.PI))) > turretSwivelConfiguration.maxVelocityTolerance())
            return false;
        if (Math.abs(this.getCurrentPosition()-this.getDesiredValue())*360 > turretSwivelConfiguration.maxAngleTolerance())
            return false;
        double angleDegrees = this.getCurrentPosition()*360;
        if (angleDegrees < turretSwivelConfiguration.lowerAngleBound() || angleDegrees > turretSwivelConfiguration.upperAngleBound())
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
