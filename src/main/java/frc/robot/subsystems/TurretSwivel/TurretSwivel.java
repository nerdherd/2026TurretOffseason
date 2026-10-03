package frc.robot.subsystems.TurretSwivel;

import static frc.robot.Constants.LoggingConstants.kSubsystemTab;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import dev.doglog.DogLog;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.nerd_math.NerdyMath;

public class TurretSwivel extends TemplateSubsystem {
    private TurretSwivelConfiguration turretSwivelConfiguration;
    public TurretSwivel(String name, int motorID, SubsystemMode mode, double defaultValue, boolean useSubsystem){
        super(name, motorID, mode, defaultValue, useSubsystem);
    }
    public TurretSwivel setTurretSwivelConfiguration(TurretSwivelConfiguration configuration) {
        turretSwivelConfiguration = configuration;
        return this;
    }
    public Pose2d getPose(Pose2d swervePosition, double headingDegrees) {
        Translation2d translation = turretSwivelConfiguration.relativePosition().getTranslation().rotateBy(Rotation2d.fromDegrees(headingDegrees));
        return swervePosition.transformBy(new Transform2d(translation, Rotation2d.ZERO));
    }
    public static double getRobotRelativeAngle(double robotHeading, double desiredAngle) {
        return desiredAngle - robotHeading;
    }
    public boolean isReadyToShoot(){
        if (Math.abs(this.getCurrentVelocity()) > turretSwivelConfiguration.maxVelocityTolerance())
            return false;
        if (Math.abs(this.getCurrentPosition()-this.getDesiredValue())*360 > turretSwivelConfiguration.maxAngleTolerance())
            return false;
        double angleDegrees = this.getCurrentPosition()*360;
        if (angleDegrees < turretSwivelConfiguration.lowerAngleBound() || angleDegrees > turretSwivelConfiguration.upperAngleBound())
            return false;
        return true;
    }
    /**
     * 
     * @param desiredAngleDegrees Robot Relative Degrees
     */
    public void goToAngle(double desiredAngleDegrees){
        desiredAngleDegrees = NerdyMath.posMod(desiredAngleDegrees, 360);
        if (desiredAngleDegrees < turretSwivelConfiguration.lowerAngleBound() || desiredAngleDegrees > turretSwivelConfiguration.upperAngleBound())
            return;
        this.setDesiredValue(desiredAngleDegrees/360.0);
    }
    public Command goToAngleCommand(double desiredAngleDegrees) {
        return Commands.runOnce(() -> goToAngle(desiredAngleDegrees));
    }

    public Pose2d getRelativePose(){
        return turretSwivelConfiguration.relativePosition();
    }

    /** applies configuration to motors; should be used on construction */
    @Override
	public TurretSwivel configureMotors(TalonFXConfiguration configuration){
		this.configuration = configuration;
		DogLog.log(kSubsystemTab + name + "/motor configs", configuration.toString());
		applyMotorConfigs();
		return this;
	}
}
