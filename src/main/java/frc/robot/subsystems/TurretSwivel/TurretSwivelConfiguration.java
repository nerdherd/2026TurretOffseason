package frc.robot.subsystems.TurretSwivel;

import org.wpilib.math.geometry.Pose2d;

public record TurretSwivelConfiguration(double deadbandMinDegrees, double deadbandMaxDegrees, Pose2d relativePosition, double maxAngleTolerance, double maxVelocityTolerance) {
    public TurretSwivelConfiguration(){
        this(0, 0, Pose2d.kZero, 0, 0);
    }    
    public TurretSwivelConfiguration setDeadbandMin(double deadbandMinDegrees){
        return new TurretSwivelConfiguration(deadbandMinDegrees, deadbandMaxDegrees, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    } 
    public TurretSwivelConfiguration setDeadbandMax(double deadbandMaxDegrees){
        return new TurretSwivelConfiguration(deadbandMinDegrees, deadbandMaxDegrees, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    } 
    public TurretSwivelConfiguration setRelativePosition(Pose2d relativePosition){
        return new TurretSwivelConfiguration(deadbandMinDegrees, deadbandMaxDegrees, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    }
    public TurretSwivelConfiguration setMaxAngleTolerance(double maxAngleTolerance){
        return new TurretSwivelConfiguration(deadbandMinDegrees, deadbandMaxDegrees, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    } 
    public TurretSwivelConfiguration setMaxVelocityTolerance(double maxVelocityTolerance){
        return new TurretSwivelConfiguration(deadbandMinDegrees, deadbandMaxDegrees, relativePosition, maxAngleTolerance, maxVelocityTolerance);
    } 
}
