package frc.robot.subsystems.TurretSwivel;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.util.NerdyMath;

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
        return swervePosition.transformBy(new Transform2d(translation, Rotation2d.kZero));
    }
    public static double getRobotRelativeAngle(double robotHeading, double desiredAngle) {
        return desiredAngle - robotHeading;
    }
    public boolean isReadyToShoot(){
        if (Math.abs(this.getCurrentVelocity()) > turretSwivelConfiguration.maxVelocityTolerance())
            return false;
        if (Math.abs(this.getCurrentPosition()-this.getDesiredValue())*360 > turretSwivelConfiguration.maxAngleTolerance())
            return false;
        if ((this.getCurrentPosition()*360) > turretSwivelConfiguration.deadbandMinDegrees() && (this.getCurrentPosition()*360) < turretSwivelConfiguration.deadbandMaxDegrees())
            return false;
        return true;
    }
    /**
     * 
     * @param desiredAngleDegrees Robot Relative Degrees
     */
    public void goToAngle(double desiredAngleDegrees){
        desiredAngleDegrees = NerdyMath.posMod(desiredAngleDegrees, 360);
        if (desiredAngleDegrees > turretSwivelConfiguration.deadbandMinDegrees() && desiredAngleDegrees < turretSwivelConfiguration.deadbandMaxDegrees())
            return;
        this.setDesiredValue(desiredAngleDegrees/360.0);
    }
    public Command goToAngleCommand(double desiredAngleDegrees) {
        return Commands.runOnce(() -> goToAngle(desiredAngleDegrees));
    }
}
